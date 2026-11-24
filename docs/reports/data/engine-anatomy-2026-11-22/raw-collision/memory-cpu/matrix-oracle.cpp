// Cross-check sparse coefficients against the quadratic energy of independent impulses.
#include <dynamics/event_driven/ContactSystem.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace kp;
namespace {
struct Pair { int a, b; };
int check(const char* name, const std::vector<Body>& bodies, std::initializer_list<Pair> pairs) {
    StepArray<ContactSystem::Line> rows;
    for (Pair pair : pairs) {
        const double t=double(rows.size()+1);
        rows.push_back({pair.a,pair.b,{1/t,-t/3,0.125},{-1.5/t,0.25,t},{0.5,-t,t/7}});
    }
    const auto matrix=ContactSystem::responseMatrix(bodies,rows);
    if (matrix.size()!=rows.size()) return 1;
    std::vector<Vector3> linear(bodies.size()),angular(bodies.size());
    double q=0;
    size_t terms=0;
    for (size_t i=0; i<rows.size(); ++i) {
        const auto& r=rows[i];
        const double v=double(i+1)/(i%2 ? -3.0 : 2.0);
        linear[r.a]-=v*r.linear; linear[r.b]+=v*r.linear;
        angular[r.a]-=v*r.angularA; angular[r.b]+=v*r.angularB;
        size_t previous=0;
        bool first=true;
        for (const auto& e:matrix[i]) {
            if (e.column>=rows.size() || (!first && e.column<=previous)) return 2;
            const double w=double(e.column+1)/(e.column%2 ? -3.0 : 2.0);
            q+=v*e.value*w;
            std::printf("%s,%zu,%zu,%a\n",name,i,e.column,e.value);
            previous=e.column; first=false; ++terms;
        }
    }
    double energy=0;
    for (size_t i=0;i<bodies.size();++i) {
        const auto& b=bodies[i];
        if (b.isStatic()) continue;
        energy+=b.mass.inverseMass*linear[i].dot(linear[i]);
        // In principal axes the angular contribution is the sum of three squares divided by I.
        const Vector3 local=b.orientation.rotateInverse(angular[i]);
        energy+=local.dot(b.mass.inverseInertia.scaled(local));
    }
    const double relative=std::abs(energy-q)/std::max(1.0,energy);
    std::fprintf(stderr,"%s,%zu,%zu,%zu,%.14g\n",name,bodies.size(),rows.size(),terms,relative);
    return std::isfinite(relative) && relative<1e-12 ? 0 : 3;
}
}
int main() {
    StepMemory memory;
    const auto make=[](double density) {
        auto b=Body::box({.4,.7,.9},density);
        b.orientation=Quaternion{1,2,3,-1}.unit();
        return b;
    };
    const std::vector<Body> bodies{Body{},make(1e-4),make(1),make(1e4),Body{}};
    std::puts("case,row,column,hex_coefficient");
    std::fputs("case,bodies,rows,entries,relative_energy_error\n",stderr);
    return check("empty",{}, {}) | check("empty-working-set",bodies,{})
         | check("static",bodies,{{0,4},{4,0}})
         | check("common-fixed-anchor",bodies,{{0,1},{0,2},{0,3}})
         | check("repeated-pair",bodies,{{1,2},{1,2},{2,1},{2,1}})
         | check("rotated-mixed-masses",bodies,{{0,1},{1,3},{3,2},{2,0},{1,2},{3,1}});
}
