// Собирается отдельно вместо grip_jolt.cpp, с теми же флагами цели Jolt.
#define main joltGripMain
#include "../../engines/grip_jolt.cpp"
#undef main
#include <Jolt/Core/Memory.h>
#include <cinttypes>
#include <cstdlib>

#ifdef JPH_DISABLE_CUSTOM_ALLOCATOR
#error "This probe requires a Jolt library built with DISABLE_CUSTOM_ALLOCATOR=OFF."
#else
namespace {
// Один мир и JobSystemSingleThreaded. Сами счётчики не выделяют память.
// Для Free и AlignedFree известен только адрес. Пиковый остаток из этих данных не вычисляется.
struct Requests {
    std::uint64_t allocateCalls = 0, allocateBytes = 0;
    std::uint64_t alignedCalls = 0, alignedBytes = 0;
    std::uint64_t reallocCalls = 0, reallocNullCalls = 0, reallocOldBytes = 0, reallocNewBytes = 0;
    std::uint64_t freeCalls = 0, freeNullCalls = 0, alignedFreeCalls = 0, alignedFreeNullCalls = 0;
};
Requests phase;

struct Allocator {
    JPH::AllocateFunction allocate;
    JPH::ReallocateFunction reallocate;
    JPH::FreeFunction free;
    JPH::AlignedAllocateFunction alignedAllocate;
    JPH::AlignedFreeFunction alignedFree;
};
Allocator original;

void* countAllocate(std::size_t size) {
    ++phase.allocateCalls;
    phase.allocateBytes += size;
    return original.allocate(size);
}

void* countReallocate(void* block, std::size_t oldSize, std::size_t newSize) {
    ++phase.reallocCalls;
    phase.reallocNullCalls += block == nullptr;
    phase.reallocOldBytes += oldSize;
    phase.reallocNewBytes += newSize;
    return original.reallocate(block, oldSize, newSize);
}

void countFree(void* block) {
    ++phase.freeCalls;
    phase.freeNullCalls += block == nullptr;
    original.free(block);
}

void* countAligned(std::size_t size, std::size_t alignment) {
    ++phase.alignedCalls;
    phase.alignedBytes += size;
    return original.alignedAllocate(size, alignment);
}

void countAlignedFree(void* block) {
    ++phase.alignedFreeCalls;
    phase.alignedFreeNullCalls += block == nullptr;
    original.alignedFree(block);
}

struct ObserveAllocator {
    ObserveAllocator() {
        original = {JPH::Allocate, JPH::Reallocate, JPH::Free, JPH::AlignedAllocate, JPH::AlignedFree};
        JPH::Allocate = countAllocate;
        JPH::Reallocate = countReallocate;
        JPH::Free = countFree;
        JPH::AlignedAllocate = countAligned;
        JPH::AlignedFree = countAlignedFree;
    }
    ~ObserveAllocator() {
        JPH::Allocate = original.allocate;
        JPH::Reallocate = original.reallocate;
        JPH::Free = original.free;
        JPH::AlignedAllocate = original.alignedAllocate;
        JPH::AlignedFree = original.alignedFree;
    }
    ObserveAllocator(const ObserveAllocator&) = delete;
};

void report(const char* name, int steps) {
    std::printf("%s,%d,%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64
                ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 "\n",
                name, steps, phase.allocateCalls, phase.allocateBytes, phase.alignedCalls, phase.alignedBytes,
                phase.reallocCalls, phase.reallocNullCalls, phase.reallocOldBytes, phase.reallocNewBytes,
                phase.freeCalls, phase.freeNullCalls, phase.alignedFreeCalls, phase.alignedFreeNullCalls);
    phase = {};
}

void runFrames(Rig& rig, double squeeze, double down) {
    for (int frame = 0; frame < 15; ++frame)
        if (!rig.step(1.0 / 60, squeeze, down)) std::abort(); // 15 × 8 вызовов PhysicsSystem::Update
}
}

int main() {
    JoltRuntime runtime;
    std::fputs("baseline=after_JoltRuntime_registration; scope=single_world\n", stderr);
    std::puts("phase,public_steps,alloc_calls,alloc_request_B,aligned_alloc_calls,aligned_request_B,"
              "realloc_calls,realloc_null_calls,realloc_old_B,realloc_request_B,"
              "free_calls,free_null_calls,aligned_free_calls,aligned_free_null_calls");
    ObserveAllocator observe;
    {
        constexpr grip::Load load{0.5, 10, 10};
        Rig rig(load);
        report("create", 0);
        runFrames(rig, load.squeeze, 0);
        report("preload", 120);
        const auto initial = rig.read();
        phase = {};
        runFrames(rig, load.squeeze, load.down);
        report("loaded", 120);
        const auto final = rig.read();
        if (!initial.valid() || !final.valid()) std::abort();
        std::fprintf(stderr, "end_drift_mm=%.9g\n", 1000 * std::abs(final.relativeY - initial.relativeY));
        phase = {};
    }
    report("destroy", 0);
    return 0;
}
#endif
