// A rail joins a prescribed support and a moving body. Check a nonzero impulse
// small enough to disprove any epsilon cutoff, and both signs of zero.
#include <dynamics/time_stepping/DirectJoints.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>
using namespace kp;
static std::uint64_t bits(Real v) {
    std::uint64_t u;
    std::memcpy(&u,&v,sizeof u);
    return u;
}
int main() {
    StepMemory step;
    std::puts("initial,lambda,axis,expected,actual_hex,expected_hex,static_v,correct");
    for (Real initial : {0., -0., .375}) for (Real impulse : {0., -0., 0x1p-400, -0x1p-400, 2., -2.})
        for (Real axis : {1., -1.}) {
            StepArray<SolverBody> bodies(2);
            bodies[0].v.x = .625; // prescribed support; no impulse response
            bodies[1].v.x = initial;
            bodies[1].inverseMass = .5;
            DirectJoints joints;
            joints.add(Bond{0,1,{axis,0,0},{},{},0,0},&impulse);
            joints.assemble(bodies);
            joints.warmStart(bodies);
            const Real expected=initial+(.5*impulse)*axis;
            const Real actual=bodies[1].v.x;
            const bool correct=actual==expected && bodies[0].v.x==.625;
            std::printf("%a,%a,%g,%a,%016llx,%016llx,%a,%d\n",initial,impulse,axis,expected,
                        static_cast<unsigned long long>(bits(actual)),static_cast<unsigned long long>(bits(expected)),bodies[0].v.x,int(correct));
            if (!correct) return 1;
        }
}
