// A diagnostic executable, linked on the measurement host. Never use its timings.
#define main originalGripMain
#include "GripLimit.cpp"
#undef main
#include "MemoryCounters.h"
#include <math/StepMemory.h>
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <psapi.h>

int main() {
    kp::WorkerPool::shared().resize(1);
    constexpr grip::Load load{0.5, 10, 10};
    constexpr int worlds = 128, frames = 15;
    std::puts("world,phase,frames,new_calls,new_bytes,arena_issues,payload_B,occupied_B,refills,refill_B,peak_B,blocks,held_after_B,drift_mm,private_B,working_set_B");
    for (int n = 1; n <= worlds; ++n) {
        double drift = 0;
        {
            Rig rig(1, load, true);
            rig.world().soft.coupledFriction = true;
            for (int phase = 0; phase < 2; ++phase) {
                const double initial = rig.read().relativeY;
                memory_study::start();
                for (int i = 0; i < frames; ++i) {
                    if (!rig.step(1.0 / 60, load.squeeze, phase ? load.down : 0)) return 2;
                    if (kp::StepMemory::held() != 0) return 3;
                }
                const auto counts = memory_study::stop();
                const auto reading = rig.read();
                if (!reading.valid()) return 4;
                drift = (initial - reading.relativeY) * 1000;
                std::printf("%d,%s,%d,%zu,%zu,%zu,%zu,%zu,%zu,%zu,%zu,%zu,%zu,%.12g,,\n",
                    n, phase ? "loaded" : "preload", frames, counts.heapCalls, counts.heapBytes,
                    counts.arenaCalls, counts.payload, counts.occupied, counts.refills,
                    counts.refillBytes, counts.peak, counts.blocks, kp::StepMemory::held(), drift);
            }
        }
        PROCESS_MEMORY_COUNTERS_EX memory{};
        memory.cb = sizeof(memory);
        if (!GetProcessMemoryInfo(GetCurrentProcess(),
                reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&memory), sizeof(memory))) return 5;
        std::printf("%d,destroyed,0,,,,,,,,,,%zu,%.12g,%zu,%zu\n",
            n, kp::StepMemory::held(), drift, size_t(memory.PrivateUsage), size_t(memory.WorkingSetSize));
        if (kp::StepMemory::held() != 0) return 6;
    }
}
