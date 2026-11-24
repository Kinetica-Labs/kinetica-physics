// Кинематические позы двух выпуклых тел, м. Оракул проецирует перечисленные вершины на все SAT-оси.
// Проверяется narrow phase в этих позах. Время непрерывного удара измеряет отдельный прибор.
#include "HullSweepShapes.h"
#include <collision/ContactPair.h>
#include <collision/narrowphase/HullVsHullAlgorithm.h>
#include <collision/narrowphase/Sat.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>
#include <map>
#include <string>
#include <utility>

namespace {
using namespace kp;
using hull_sweep::Mesh;
constexpr double oracleTolerance = 1e-9, supportTolerance = 1e-9, witnessTolerance = 1e-7;
const char* const names[] = {"vertex","edge","face"};
struct V { long double x=0,y=0,z=0; };
V operator+(V a,V b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
V operator-(V a,V b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
V operator*(long double k,V a) { return {k*a.x,k*a.y,k*a.z}; }
long double dot(V a,V b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
V cross(V a,V b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
long double norm(V v) { return std::sqrt(dot(v,v)); }
V unit(V v) { return (1/norm(v))*v; }
V v(Vector3 a) { return {a.x,a.y,a.z}; }
Vector3 sdk(V a) { return {Real(a.x),Real(a.y),Real(a.z)}; }

// Независимое преобразование выписано через компоненты кватерниона. Без GJK/SAT/поддержки SDK.
V worldPoint(Transform t,V x) {
    const auto q = t.orientation;
    const V u{q.x,q.y,q.z};
    const auto b = 2*cross(u,x);
    return v(t.position)+x+q.w*b+cross(u,b);
}
struct Edge { int a,b; };
struct Feature {
    int type,id;
    std::vector<int> vertices;
    V centre,normal; // центр внутри элемента и направление строго внутри его нормального конуса
};
struct Solid {
    Mesh mesh;
    std::vector<Edge> edges;
    std::vector<Feature> features;
    Body body;

    explicit Solid(Mesh shape) : mesh(std::move(shape)) {
        const int n = int(mesh.vertices.size());
        std::vector<V> vertexNormals(n),faceNormals;
        std::map<std::pair<int,int>, V> edgeNormals;
        for (const auto& f : mesh.faces) {
            const V p = v(mesh.vertices[f[0]]);
            const V normal = unit(cross(v(mesh.vertices[f[1]])-p,v(mesh.vertices[f[2]])-p));
            faceNormals.push_back(normal);
            for (size_t k=0; k<f.size(); ++k) {
                const int a=f[k], b=f[(k+1)%f.size()];
                vertexNormals[a] = vertexNormals[a]+normal;
                const std::pair<int,int> key = std::minmax(a,b);
                edgeNormals[key] = edgeNormals[key]+normal;
            }
        }
        const auto add = [&](int type,int id,std::vector<int> vertices,V normal) {
            V centre;
            for (int i : vertices) centre = centre+v(mesh.vertices[i]);
            centre = (1.L/vertices.size())*centre;
            std::sort(vertices.begin(),vertices.end());
            features.push_back({type,id,std::move(vertices),centre,unit(normal)});
        };
        for (int i=0; i<n; ++i) add(0,i,{i},vertexNormals[i]);
        // Индекс ребра — порядок неориентированной пары вершин (min,max) в этой таблице.
        for (const auto& [key,normal] : edgeNormals) {
            add(1,int(edges.size()),{key.first,key.second},normal);
            edges.push_back({key.first,key.second});
        }
        for (size_t i=0; i<mesh.faces.size(); ++i) add(2,int(i),mesh.faces[i],faceNormals[i]);
        body.shape = std::make_shared<ConvexHull>(mesh.vertices,mesh.faces);
        body.mass = MassProperties::box({1,1,1},1); // здесь есть только запрос геометрии, без интегрирования
    }
};
struct Posed {
    std::vector<V> points,normal,edge;
    Posed(const Solid& shape,Transform pose) {
        for (auto p : shape.mesh.vertices) points.push_back(worldPoint(pose,v(p)));
        for (const auto& f : shape.mesh.faces)
            normal.push_back(unit(cross(points[f[1]]-points[f[0]],points[f[2]]-points[f[0]])));
        for (auto e : shape.edges) edge.push_back(points[e.b]-points[e.a]);
    }
};
struct Projection { long double lo,hi; };
Projection project(const std::vector<V>& points,V axis) {
    const long double first=dot(points[0],axis);
    Projection range{first,first};
    for (auto p : points) {
        const long double x=dot(p,axis);
        range.lo=std::min(range.lo,x); range.hi=std::max(range.hi,x);
    }
    return range;
}
struct Sat {
    long double gap=-std::numeric_limits<long double>::infinity();
    int axes=0, axisA=-1,axisB=-1;
    V direction;
};
Sat oracle(const Posed& a,const Posed& b) {
    Sat result;
    const auto axis = [&](V n,int ia,int ib) {
        const long double d=norm(n);
        if (d == 0) return; // только нулевая ось не задаёт проекции
        const auto x=project(a.points,n), y=project(b.points,n);
        const long double forward=y.lo-x.hi, backward=x.lo-y.hi, gap=std::max(forward,backward)/d;
        if (gap>result.gap) {
            result.gap=gap; result.axisA=ia; result.axisB=ib;
            result.direction=((forward>=backward ? 1.L : -1.L)/d)*n;
        }
        ++result.axes;
    };
    for (size_t i=0; i<a.normal.size(); ++i) axis(a.normal[i],int(i),-1);
    for (size_t j=0; j<b.normal.size(); ++j) axis(b.normal[j],-1,int(j));
    for (size_t i=0; i<a.edge.size(); ++i) for (size_t j=0; j<b.edge.size(); ++j)
        axis(cross(a.edge[i],b.edge[j]),int(i),int(j));
    return result;
}
std::vector<int> support(const Posed& p,V n) {
    const long double limit=project(p.points,n).hi;
    std::vector<int> indices;
    for (size_t i=0; i<p.points.size(); ++i)
        if (limit-dot(p.points[i],n) <= supportTolerance) indices.push_back(int(i));
    return indices;
}
int featureIndex(const Solid& s,const std::vector<int>& set) {
    for (size_t i=0; i<s.features.size(); ++i) if (s.features[i].vertices==set) return int(i);
    return -1;
}
// Точка на границе выпуклого тела удовлетворяет max_i n_i·(x−x_i)=0.
double boundaryError(const Solid& s,const Posed& p,V x) {
    long double furthest=-std::numeric_limits<long double>::infinity();
    for (size_t i=0; i<s.mesh.faces.size(); ++i)
        furthest=std::max(furthest,dot(p.normal[i],x-p.points[s.mesh.faces[i][0]]));
    return double(std::abs(furthest));
}
Quaternion around(V axis,double angle) {
    return Quaternion::fromRotationVector(Real(angle)*sdk(unit(axis)));
}
Quaternion align(V from,V to) {
    const long double d=dot(from,to);
    if (d < -1+1e-14L) {
        const V basis=std::abs(from.x)<0.8 ? V{1,0,0} : V{0,1,0};
        return around(cross(from,basis),std::acos(-1.));
    }
    const V c=cross(from,to);
    return Quaternion{Real(1+d),Real(c.x),Real(c.y),Real(c.z)}.unit();
}
const V normal=unit({0.31,-0.45,0.83}), lateral=unit(cross(normal,{0,0,1}));
const V sideways=cross(normal,lateral);
struct Phase { const char* name; double t,gap,tilt; int expected; };
// Последняя пара поз — малые возмущения по обе стороны истинного касания (touch_in).
// Там тип и знак встречи выводятся оракулом. Центры выбранных элементов идут непрерывными путями.
constexpr Phase phases[]={{"approach",0,0.003,0,1},{"touch_in",0.25,0,0,0},{"overlap",0.5,-0.0001,0,-1},
                          {"touch_out",0.75,0,0,0},{"release",1,0.003,0,1},
                          {"tilt_minus",0.25,0,-1,2},{"tilt_plus",0.25,0,1,2}};
struct Placement { Transform a,b; V anchor; };
Placement place(const Feature& a,const Feature& b,int twist,Phase p) {
    const V anchor=V{0.1,-0.16,0.32}+(0.04*p.t)*lateral+(0.017*p.t)*sideways;
    const auto qa=(around(sideways,0.008*p.tilt)*around(normal,0.33+0.17*p.t)*align(a.normal,normal)).unit();
    const auto qb=(around(lateral,-0.011*p.tilt)*around(normal,0.72+0.91*twist-0.23*p.t)*align(b.normal,-1*normal)).unit();
    const V offset=(0.5*p.gap)*normal+(0.0006*p.tilt)*lateral;
    return {{sdk(anchor-offset)-qa.rotate(sdk(a.centre)),qa},{sdk(anchor+offset)-qb.rotate(sdk(b.centre)),qb},anchor};
}
void printPose(Transform t) {
    std::printf(",%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g",double(t.position.x),double(t.position.y),
                double(t.position.z),double(t.orientation.w),double(t.orientation.x),double(t.orientation.y),double(t.orientation.z));
}
void geometry(const char* name,const Solid& s) {
    for (size_t f=0; f<s.mesh.faces.size(); ++f) for (size_t k=0; k<s.mesh.faces[f].size(); ++k) {
        const int i=s.mesh.faces[f][k];
        const auto x=s.mesh.vertices[i];
        std::printf("%s,%zu,%zu,%d,%.17g,%.17g,%.17g\n",name,f,k,i,double(x.x),double(x.y),double(x.z));
    }
}
void animation(const Solid& a,const Solid& b) {
    std::puts("type_a,id_a,type_b,id_b,frame,t,gap_m,ax_m,ay_m,az_m,aqw,aqx,aqy,aqz,bx_m,by_m,bz_m,bqw,bqx,bqy,bqz,"
              "witness_ax_m,witness_ay_m,witness_az_m,witness_bx_m,witness_by_m,witness_bz_m,nx,ny,nz");
    for (const auto& fa:a.features) for (const auto& fb:b.features) if (!fa.id && !fb.id) {
        for (int frame=0; frame<=24; ++frame) {
            const int segment=std::min(frame/6,3);
            const double alpha=double(frame-6*segment)/6, t=double(frame)/24;
            const double g=(1-alpha)*phases[segment].gap+alpha*phases[segment+1].gap;
            const auto p=place(fa,fb,0,{"animation",t,g,0,2});
            const V wa=worldPoint(p.a,fa.centre),wb=worldPoint(p.b,fb.centre);
            std::printf("%s,%d,%s,%d,%d,%.17g,%.17g",names[fa.type],fa.id,names[fb.type],fb.id,frame,t,g);
            printPose(p.a); printPose(p.b);
            std::printf(",%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n",
                double(wa.x),double(wa.y),double(wa.z),double(wb.x),double(wb.y),double(wb.z),double(normal.x),double(normal.y),double(normal.z));
        }
    }
}

// Точный ключ из сетки. Одни и те же мировые позы в обеих перестановках, без повторной подгонки.
bool diagnoseOrder(const char* order,Solid& a,Solid& b,Transform ta,Transform tb) {
    a.body.position=ta.position; a.body.orientation=ta.orientation;
    b.body.position=tb.position; b.body.orientation=tb.orientation;
    const Posed A(a,ta),B(b,tb);
    const auto independent=oracle(A,B);
    const auto& ha=*a.body.shape->hull(); const auto& hb=*b.body.shape->hull();
    const TransformedHull localA(ha,Transform{}), localB(hb,ta.relative(tb));
    const double na=std::numeric_limits<double>::quiet_NaN();
    const auto row=[&](const char* stage,const char* branch,double margin,double preference,double touching,
                       V n,double nativeGap,int featureA,int featureB,int count,int index,long long id,
                       double sep,V onA,V onB,int cores) {
        const double gap=double(project(B.points,n).lo-project(A.points,n).hi);
        std::printf("%s,%s,%s,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,"
                    "%d,%d,%d,%d,%lld,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%d\n",
                    order,stage,branch,margin,preference,touching,double(n.x),double(n.y),double(n.z),
                    nativeGap,gap,double(independent.gap),featureA,featureB,count,index,id,sep,
                    double(onA.x),double(onA.y),double(onA.z),double(onB.x),double(onB.y),double(onB.z),
                    double(dot(n,onB-onA))-sep,cores);
    };
    const V empty{na,na,na};
    const auto satKind=[](SeparatingAxis::Kind k) {
        return k==SeparatingAxis::Kind::FaceA ? "face_a" : k==SeparatingAxis::Kind::FaceB ? "face_b" : "edge_pair";
    };
    const auto axisRow=[&](const char* label,const SeparatingAxis& axis,Real preference) {
        row(label,satKind(axis.kind),na,preference,na,v(ta.direction(axis.normal)),axis.separation,axis.featureA,
            axis.featureB,0,-1,-1,na,empty,empty,-1);
    };
    row("all_vertex_sat",independent.axisA<0 ? "face_b" : independent.axisB<0 ? "face_a" : "edge_pair",
        na,0,na,independent.direction,double(independent.gap),independent.axisA,independent.axisB,0,-1,-1,na,empty,empty,-1);
    const auto gjk=closestPoints(*a.body.shape,ta,*b.body.shape,tb,kContactDistance);
    row("gjk",gjk.beyond?"beyond":"closest",kContactDistance,na,na,v(gjk.normal),gjk.distance,-1,-1,1,0,-1,
        gjk.distance,v(gjk.onA),v(gjk.onB),int(gjk.coresOverlap));
    for (Real pref : {Real(0),kOnBoundary}) {
        axisRow("sdk_best_face",findFaceAxis(localA,localB,pref),pref);
        axisRow("sdk_separating_axis",findSeparatingAxis(localA,localB,pref),pref);
    }
    const auto emit=[&](const char* label,const ContactManifold& m,double touching,double gap) {
        const char* path=m.count==0 ? "empty" : m.faceOnB ? "face_b" :
            (m.points[0].id&0xc0000000u)==0xc0000000u ? "edge_pair" : "face_a_or_fallback";
        if (!m.count)
            row(label,path,kContactDistance,kOnBoundary,touching,v(m.normal),gap,-1,-1,0,-1,-1,na,empty,empty,-1);
        for (int i=0; i<m.count; ++i) {
            const auto& p=m.points[i];
            row(label,path,kContactDistance,kOnBoundary,touching,v(m.normal),gap,-1,-1,m.count,i,p.id,
                p.separation,v(p.onA),v(p.onB),-1);
        }
    };
    for (Real touching : {Real(0),kOnBoundary,kContactDistance})
        emit("sdk_clip",HullVsHullAlgorithm::clipFaces(ha,ta,hb,tb,kContactDistance,touching,kOnBoundary),touching,na);
    const auto hit=examinePairWithin(a.body,b.body,0,kOnBoundary);
    if (!hit) return false;
    const auto& m=hit->manifold;
    emit("actual_manifold",m,kContactDistance,hit->distance);
    const auto n=v(m.normal);
    const double supportGap=double(project(B.points,n).lo-project(A.points,n).hi);
    const bool pass=std::abs(independent.gap)<=oracleTolerance && m.count>0 &&
                    std::abs(norm(n)-1)<=1e-8 && std::abs(supportGap)<=witnessTolerance;
    std::fprintf(stderr,"order=%s oracle_gap_m=%.17g sdk_support_gap_m=%.17g tolerance_m=%.9g pass=%d\n",
                 order,double(independent.gap),supportGap,witnessTolerance,int(pass));
    return pass;
}
bool diagnose(Solid& a,Solid& b) {
    const auto fa=std::find_if(a.features.begin(),a.features.end(),[](const Feature& f){return f.type==1 && f.id==13;});
    const auto fb=std::find_if(b.features.begin(),b.features.end(),[](const Feature& f){return f.type==0 && f.id==1;});
    if (fa==a.features.end() || fb==b.features.end()) return false;
    const auto p=place(*fa,*fb,1,phases[3]);
    std::puts("order,stage,branch,margin_m,axis_preference_m,touching_m,nx,ny,nz,stage_gap_m,"
              "independent_gap_along_output_normal_m,independent_max_gap_m,feature_a,feature_b,point_count,point_index,"
              "point_id,point_separation_m,on_ax_m,on_ay_m,on_az_m,on_bx_m,on_by_m,on_bz_m,point_gap_residual_m,gjk_cores_overlap");
    const bool ab=diagnoseOrder("AB",a,b,p.a,p.b);
    const bool ba=diagnoseOrder("BA",b,a,p.b,p.a);
    return ab && ba;
}
struct Tally { int cases=0,fixtureFailures=0,physicsFailures=0,switched=0,overlap=0,touch=0,separate=0; };
bool measure(Solid& a,Solid& b,const Feature& fa,const Feature& fb,int twist,Phase phase,Tally& tally,
             bool onlyFailures,bool swapped) {
    const auto pose=place(fa,fb,twist,phase);
    a.body.position=pose.a.position; a.body.orientation=pose.a.orientation;
    b.body.position=pose.b.position; b.body.orientation=pose.b.orientation;
    const Posed A(a,pose.a), B(b,pose.b);
    const auto result=oracle(A,B);
    const int state=result.gap > oracleTolerance ? 1 : result.gap < -oracleTolerance ? -1 : 0;
    const auto supportA=support(A,normal), supportB=support(B,-1*normal);
    const int ia=featureIndex(a,supportA), ib=featureIndex(b,supportB);
    const bool same=supportA==fa.vertices && supportB==fb.vertices;
    const V ca=worldPoint(pose.a,fa.centre), cb=worldPoint(pose.b,fb.centre);
    const double centreResidual=double(norm((cb-ca)-(phase.gap*normal+0.0012*phase.tilt*lateral)));
    const double witness=std::max(boundaryError(a,A,ca),boundaryError(b,B,cb));
    const bool nominal=phase.tilt==0;
    const bool fixture=std::isfinite(double(result.gap)) && ia>=0 && ib>=0 && centreResidual<=supportTolerance &&
        witness<=supportTolerance && (!nominal || (same && state==phase.expected));
    const auto hit=swapped ? examinePairWithin(b.body,a.body,0,kOnBoundary) : examinePairWithin(a.body,b.body,0,kOnBoundary);
    const int points=hit && hit->inContact() ? hit->manifold.count : 0;
    // Между истинным касанием и границей kContactDistance разрешена как ранняя площадка, так и её отсутствие.
    const bool shouldHit=state<=0;
    const bool far=double(result.gap)>double(kContactDistance)+oracleTolerance;
    bool valid=(!shouldHit || points>0) && (!far || points==0);
    double gapResidual=0,surfaceResidual=0,normalGap=0,normalLength=0;
    if (points) {
        const auto& m=hit->manifold;
        // Выход обеих перестановок выражен в одном соглашении: нормаль от исходного A к исходному B.
        const V n=(swapped ? -1.L : 1.L)*v(m.normal);
        normalLength=double(norm(n));
        normalGap=double(project(B.points,n).lo-project(A.points,n).hi);
        valid &= std::isfinite(normalLength) && std::abs(normalLength-1)<=1e-8 &&
                 (state!=0 || std::abs(normalGap)<=witnessTolerance);
        for (const auto& p : m.points) {
            const V onA=v(swapped ? p.onB : p.onA),onB=v(swapped ? p.onA : p.onB);
            const double r=double(std::abs(dot(n,onB-onA)-p.separation));
            const double surface=std::max(boundaryError(a,A,onA),boundaryError(b,B,onB));
            valid &= std::isfinite(double(p.separation)) && std::isfinite(r) && std::isfinite(surface);
            gapResidual=std::max(gapResidual,r); surfaceResidual=std::max(surfaceResidual,surface);
        }
        valid &= gapResidual<=witnessTolerance && surfaceResidual<=witnessTolerance;
    }
    ++tally.cases; tally.fixtureFailures+=!fixture; tally.physicsFailures+=!valid;
    tally.switched+=!nominal && !same && ia>=0 && ib>=0;
    tally.separate+=state==1; tally.touch+=state==0; tally.overlap+=state==-1;
    if (!onlyFailures || !fixture || !valid) {
        const auto field=[&](const Solid& s,int i) {
            if (i<0) std::printf(",unknown,-1"); else std::printf(",%s,%d",names[s.features[i].type],s.features[i].id);
        };
        std::printf("%s,%d,%s,%d,%d,%s",names[fa.type],fa.id,names[fb.type],fb.id,twist,phase.name);
        field(a,ia); field(b,ib);
        std::printf(",%d,%d,%.12g,%d,%.12g,%.12g,%d,%.12g,%.12g,%.12g,%.12g,%d,%d\n",int(same),state,double(result.gap),
            result.axes,centreResidual,witness,points,normalLength,normalGap,gapResidual,surfaceResidual,int(fixture),int(valid));
    }
    return fixture && valid;
}
}

int main(int argc,char** argv) {
    const std::string mode=argc==2 ? argv[1] : "";
    if (argc>2 || (!mode.empty() && mode!="--geometry" && mode!="--illustrations" && mode!="--fail-only" &&
                   mode!="--diagnose" && mode!="--check-case" && mode!="--animation" && mode!="--swapped")) return 2;
    Solid a(hull_sweep::shapeA()),b(hull_sweep::shapeB());
    if (mode=="--diagnose" || mode=="--check-case") {
        const bool pass=diagnose(a,b);
        return mode=="--diagnose" || pass ? 0 : 1;
    }
    if (mode=="--animation") { animation(a,b); return 0; }
    if (mode=="--geometry") {
        std::puts("shape,face,loop,vertex,x_m,y_m,z_m");
        geometry("A",a); geometry("B",b);
        return 0;
    }
    if (mode=="--illustrations") {
        std::puts("type_a,id_a,type_b,id_b,ax_m,ay_m,az_m,aqw,aqx,aqy,aqz,bx_m,by_m,bz_m,bqw,bqx,bqy,bqz,"
                  "witness_x_m,witness_y_m,witness_z_m,nx,ny,nz");
        for (const auto& fa:a.features) for (const auto& fb:b.features) if (!fa.id && !fb.id) {
            const auto p=place(fa,fb,0,phases[1]);
            std::printf("%s,%d,%s,%d",names[fa.type],fa.id,names[fb.type],fb.id);
            printPose(p.a); printPose(p.b);
            std::printf(",%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n",double(p.anchor.x),double(p.anchor.y),
                double(p.anchor.z),double(normal.x),double(normal.y),double(normal.z));
        }
        return 0;
    }
    std::puts("type_a,id_a,type_b,id_b,twist,phase,reached_type_a,reached_id_a,reached_type_b,reached_id_b,target_support,"
              "oracle_state,oracle_gap_m,tested_axes,witness_delta_error_m,witness_surface_error_m,sdk_points,sdk_normal_length,"
              "sdk_support_gap_m,max_point_gap_residual_m,max_point_surface_error_m,fixture_pass,sdk_pass");
    Tally totals[3][3]{};
    bool ok=true;
    for (const auto& fa:a.features) for (const auto& fb:b.features) for (int twist=0; twist<2; ++twist)
        for (auto p : phases) ok &= measure(a,b,fa,fb,twist,p,totals[fa.type][fb.type],mode=="--fail-only",mode=="--swapped");
    std::fputs("type_a,type_b,poses,fixture_failures,sdk_failures,tilt_samples_changed_support,separated,touching,overlap\n",stderr);
    int samples=0,changed=0;
    for (int i=0; i<3; ++i) for (int j=0; j<3; ++j) {
        const auto& n=totals[i][j];
        std::fprintf(stderr,"%s,%s,%d,%d,%d,%d,%d,%d,%d\n",names[i],names[j],n.cases,n.fixtureFailures,n.physicsFailures,
                     n.switched,n.separate,n.touch,n.overlap);
        samples+=n.cases; changed+=n.switched;
        ok &= n.cases>0 && n.separate>0 && n.touch>0 && n.overlap>0;
    }
    ok &= changed>0 && samples==int(a.features.size()*b.features.size()*2*std::size(phases));
    std::fprintf(stderr,"complete=1 order=%s shapes=%zux%zu total_poses=%d support_changes=%d long_double_digits=%d pass=%d\n",
                 mode=="--swapped" ? "BA" : "AB",a.features.size(),b.features.size(),samples,changed,
                 std::numeric_limits<long double>::digits,int(ok));
    return ok ? 0 : 1;
}
