#include <collision/narrowphase/NarrowPhaseAlgorithm.h>
#include <collision/shapes/Cylinder.h>
#include <collision/ContactPair.h>
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {
using namespace kp;

Quaternion align(Vector3 a, Vector3 b) {
    const Vector3 v = a.cross(b);
    return Quaternion{1 + a.dot(b), v.x, v.y, v.z}.unit();
}
Real surfaceDistance(const Cylinder& c, Vector3 p) {
    const Real r = std::hypot(p.x,p.z) - c.coreRadius;
    const Real h = std::abs(p.y) - c.coreHalfHeight;
    return std::hypot(std::max(r,Real(0)),std::max(h,Real(0))) +
           std::min(std::max(r,h),Real(0)) - c.radius;
}
bool run(int configuration, bool swap, Real gap) {
    const bool capToCap = configuration != 0;
    const char* kind = configuration == 2 ? "thin_lens" : (capToCap ? "flat" : "rim");
    const Cylinder sharp(.23,.16), rounded(.23,.16,.025);
    const Vector3 axis{.36,.48,.8}, radial{.8,0,.6}, up{0,1,0};
    const Vector3 rimNormal = .6 * radial + .8 * up;
    const Vector3 aNormal = capToCap ? up : rimNormal;
    const Vector3 aPoint = capToCap ? .3*sharp.coreRadius*radial + (sharp.coreHalfHeight+sharp.radius)*up :
                            sharp.coreRadius*radial + sharp.coreHalfHeight*up + sharp.radius*rimNormal;
    const Vector3 bPoint = .3*rounded.coreRadius*radial + (rounded.coreHalfHeight+rounded.radius)*up;
    const auto qa = Quaternion::fromRotationVector(.37*axis) * align(aNormal,axis);
    const auto qb = Quaternion::fromRotationVector(-.52*axis) * align(up,-axis);
    const Transform world{{-4,2,3},Quaternion{1,-2,3,5}.unit()};
    const auto ra=(world.orientation*qa).unit(), rb=(world.orientation*qb).unit();
    const Vector3 anchor=world.point({.125,-.25,.375}), normal=world.direction(axis);
    const Transform pa{anchor-ra.rotate(aPoint),ra};
    Transform pb{anchor+gap*normal-rb.rotate(bPoint),rb};
    const Real lensDepth=.001;
    if(configuration==2) {
        const Vector3 direction=ra.rotate(radial);
        pb.position=pa.position+(sharp.coreHalfHeight+sharp.radius+rounded.coreHalfHeight+rounded.radius+gap)*normal
                    +(sharp.coreRadius+rounded.coreRadius-lensDepth)*direction;
    }
    const Cylinder& a=swap?rounded:sharp; const Cylinder& b=swap?sharp:rounded;
    const Transform& ta=swap?pb:pa; const Transform& tb=swap?pa:pb;
    const auto nearest=closestPoints(a,ta,b,tb);
    const auto patch=NarrowPhaseAlgorithm::between(a,b).collide(a,ta,b,tb,nearest,kContactDistance,kContactDistance,kOnBoundary);
    constexpr Real tolerance=2e-7; // same physical length threshold as the published matrix, metres
    bool ok=patch.count>0;
    Real area=0;
    for(int i=0;i<patch.count;++i) {
        const auto& x=patch.points[i];
        const Real da=surfaceDistance(a,ta.localPoint(x.onA)), db=surfaceDistance(b,tb.localPoint(x.onB));
        const Real residual=(x.onB-x.onA-x.separation*patch.normal).length();
        const Vector3 ka=ta.localPoint(x.onA-a.radius*patch.normal), kb=tb.localPoint(x.onB+b.radius*patch.normal);
        const bool point=std::abs(da)<=tolerance && std::abs(db)<=tolerance &&
                         residual<=tolerance && std::abs(x.separation-gap)<=tolerance;
        ok=ok&&point;
        std::printf("%s,%d,%.12g,%d,%u,%d,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%d\n",kind,int(swap),
                    double(gap),patch.count,unsigned(x.id),i,double(x.separation),double(da),double(db),double(residual),double(std::hypot(ka.x,ka.z)-a.coreRadius),double(std::hypot(kb.x,kb.z)-b.coreRadius),int(point));
        // Three separated witnesses on parallel caps must enclose measurable area.
        for(int j=1;j<i;++j)
            area=std::max(area,((x.onA-patch.points[0].onA).cross(patch.points[j].onA-patch.points[0].onA)).length());
    }
    if(configuration==1) ok=ok && area>rounded.coreRadius*rounded.coreRadius;
    if(configuration==2) ok=ok && area>lensDepth*std::sqrt(rounded.coreRadius*lensDepth)/4;
    std::fprintf(stderr,"%s swap=%d gap=%.6g points=%d twice_area_m2=%.9g ok=%d\n",
                 kind,int(swap),double(gap),patch.count,double(area),int(ok));
    return ok;
}
}
int main() {
    std::puts("configuration,swap,gap_m,points,feature_id,point,separation_m,surface_a_m,surface_b_m,delta_error_m,core_radial_excess_a_m,core_radial_excess_b_m,pass_point");
    int bad=0;
    for(int flat:{0,1,2}) for(bool swap:{false,true}) for(double g:{0.,5e-5}) bad+=!run(flat,swap,g);
    return bad?1:0;
}
