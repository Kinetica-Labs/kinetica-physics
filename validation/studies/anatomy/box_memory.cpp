// Собирается отдельно вместо grip_box3d.cpp, с тем же Box3D и теми же флагами.
#define main box3dGripMain
#include "../../engines/grip_box3d.cpp"
#undef main
#include <box3d/base.h>
#include <cinttypes>
#include <cstdint>
#include <cstdlib>
#ifdef _WIN32
#include <malloc.h>
#endif

namespace {
// Единственный мир, стандартный синхронный исполнитель Box3D, один рабочий поток.
// Публичный callback получает размер после округления. Счётчики не выделяют память.
struct Phase {
    std::uint64_t allocations = 0, frees = 0;
    std::uint64_t allocatedBytes = 0, freedBytes = 0;
    std::uint64_t beginBytes = 0, peakBytes = 0;
    std::int64_t engineBegin = 0;
};
Phase phase;
std::uint64_t liveBytes = 0;

void* allocate(std::size_t size, std::int32_t alignment) {
#ifdef _WIN32
    void* memory = _aligned_malloc(size, alignment);
#else
    void* memory = std::aligned_alloc(alignment, size);
#endif
    if (!memory || reinterpret_cast<std::uintptr_t>(memory) % alignment != 0) std::abort();
    ++phase.allocations;
    phase.allocatedBytes += size;
    liveBytes += size;
    phase.peakBytes = std::max(phase.peakBytes, liveBytes);
    return memory;
}

void release(void* memory, std::size_t size) {
    if (size > liveBytes) std::abort();
    ++phase.frees;
    phase.freedBytes += size;
    liveBytes -= size;
#ifdef _WIN32
    _aligned_free(memory);
#else
    std::free(memory);
#endif
}

void beginPhase() {
    phase = {};
    phase.beginBytes = phase.peakBytes = liveBytes;
    phase.engineBegin = b3GetByteCount();
}

void finishPhase(const char* name, int steps) {
    if (phase.beginBytes + phase.allocatedBytes != phase.freedBytes + liveBytes) std::abort();
    std::printf("%s,%d,%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64
                ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRId64 ",%" PRId64 "\n",
                name, steps, phase.allocations, phase.allocatedBytes, phase.frees, phase.freedBytes,
                phase.beginBytes, phase.peakBytes, liveBytes, phase.engineBegin, b3GetByteCount());
}

void runFrames(Rig& rig, double squeeze, double down) {
    for (int frame = 0; frame < 15; ++frame)
        if (!rig.step(1.0 / 60, squeeze, down)) std::abort(); // shared8: 15 × 8 вызовов b3World_Step
}
}

int main() {
    b3SetAllocator(allocate, release);
    const auto baseline = b3GetByteCount();
    std::puts("phase,public_steps,alloc_calls,alloc_aligned_B,free_calls,free_aligned_B,"
              "live_aligned_begin_B,live_aligned_peak_B,live_aligned_end_B,engine_begin_B,engine_end_B");
    beginPhase();
    {
        constexpr grip::Load load{0.5, 10, 10};
        Rig rig(load, true);
        finishPhase("create", 0);
        beginPhase();
        runFrames(rig, load.squeeze, 0);
        finishPhase("preload", 120);
        const auto initial = rig.read();
        beginPhase();
        runFrames(rig, load.squeeze, load.down);
        finishPhase("loaded", 120);
        const auto final = rig.read();
        if (!initial.valid() || !final.valid()) std::abort();
        std::fprintf(stderr, "end_drift_mm=%.9g\n", 1000 * std::abs(final.relativeY - initial.relativeY));
        beginPhase();
    }
    finishPhase("destroy", 0);
    const bool balanced = liveBytes == 0 && b3GetByteCount() == baseline;
    b3SetAllocator(nullptr, nullptr);
    return balanced ? 0 : 1;
}
