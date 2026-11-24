// Два исторических заголовка берутся из снимков опыта (см. tilted-protocol.json).
#define ContactOverlay PreviousOverlay
#include "PreviousOverlay.h"
#undef ContactOverlay
#include "viewer/ContactOverlay.h"
#include "examples/scenes/StandardScenes.h"
#include "validation/engines/common.h"
#include <math/WorkerPool.h>
#include <array>

namespace {
using Clock = std::chrono::steady_clock;
constexpr kp::Real dt = kp::Real(1)/60;
constexpr int frames = 600;
struct Sample { double physics, markers; std::uint64_t hash; size_t marks; };

std::uint64_t fingerprint(const kp::World& world) {
    std::vector<State> states;
    for (const auto& b : world.bodies()) {
        const auto p=b.position; const auto q=b.orientation;
        const auto v=b.linearMomentum, w=b.angularMomentum;
        states.push_back({{p.x,p.y,p.z},{q.w,q.x,q.y,q.z},{v.x,v.y,v.z},{w.x,w.y,w.z}});
    }
    std::uint64_t hash=14695981039346656037ull;
    absorb(hash,states);
    return hash;
}

template<class Overlay>
void measure(const kp::World& checkpoint, Overlay* overlay, const char* name, int block) {
    kp::World w=checkpoint;
    std::array<Sample,frames> samples{};
    // Память отметок и кэш процессора прогреваются на полной секунде той же траектории.
    for (int i=-60; i<frames; ++i) {
        const auto a=Clock::now();
        w.step(dt);
        const auto b=Clock::now();
        const size_t count=overlay ? overlay->sample(w).points.size() : 0;
        const auto c=Clock::now();
        if (w.report().stalled || w.report().unfinished!=0) std::exit(3);
        if (i>=0) samples[size_t(i)]={std::chrono::duration<double,std::micro>(b-a).count(),
                                      overlay ? std::chrono::duration<double,std::micro>(c-b).count() : 0,
                                      fingerprint(w),count};
    }
    for (int i=0; i<frames; ++i) {
        const auto& s=samples[size_t(i)];
        std::printf("%d,%s,%d,%.3f,%.3f,%016llx,%zu\n",block,name,2821+i,s.physics,s.markers,
                    static_cast<unsigned long long>(s.hash),s.marks);
    }
}
}

int main() {
    kp::WorkerPool::shared().resize(1);
    auto checkpoint=bucketsOnRim();
    for (int f=0; f<46*60; ++f) checkpoint.step(dt);
    std::puts("block,mode,frame,physics_us,marker_us,state_hash,markers");
    std::array<int,3> order{0,1,2};
    for (int block=1; block<=12; ++block) {
        for (int mode : order) {
            PreviousOverlay old;
            ContactOverlay current;
            if (mode==0) measure<ContactOverlay>(checkpoint,nullptr,"physics",block);
            if (mode==1) measure(checkpoint,&old,"before",block);
            if (mode==2) measure(checkpoint,&current,"after",block);
        }
        std::next_permutation(order.begin(),order.end());
    }
}
