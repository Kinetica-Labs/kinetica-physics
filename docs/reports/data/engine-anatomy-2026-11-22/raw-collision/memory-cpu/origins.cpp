#define main originalGripMain
#include "GripLimit.cpp"
#undef main
#include "MemoryCounters.h"

void writeHeapOrigins();
int main() {
    kp::WorkerPool::shared().resize(1);
    constexpr grip::Load load{0.5,10,10};
    Rig rig(1, load, true);
    rig.world().soft.coupledFriction = true;
    for (int i = 0; i < 15; ++i) if (!rig.step(1.0/60,10,0)) return 1;
    memory_study::start();
    for (int i = 0; i < 15; ++i) if (!rig.step(1.0/60,10,10)) return 1;
    const auto counts = memory_study::stop();
    std::fprintf(stderr,"%zu calls, %zu bytes\n", counts.heapCalls, counts.heapBytes);
    writeHeapOrigins();
}
