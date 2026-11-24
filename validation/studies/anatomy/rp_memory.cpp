// Windows probe for RP3D's public base allocator with the original shared8 grip scene.
// The heap is backed by this allocator. The pool and single-frame allocator use that heap.
#define KP_RP3D_MEMORY_STUDY
#include "../../engines/grip_rp3d.cpp"
#include <array>
#include <cstdint>
#include <new>

namespace {
struct Usage {
    std::size_t calls = 0, bytes = 0, frees = 0, freedBytes = 0;
};

enum Phase { construction, preload, loaded, destruction };

class Counter final : public reactphysics3d::MemoryAllocator {
public:
    Phase phase = construction;
    std::array<Usage, 4> usage{};

private:
    void* allocate(std::size_t bytes) override {
        void* ptr = _aligned_malloc(bytes, reactphysics3d::GLOBAL_ALIGNMENT);
        if (!ptr) throw std::bad_alloc();
        if (reinterpret_cast<std::uintptr_t>(ptr) % reactphysics3d::GLOBAL_ALIGNMENT != 0) std::abort();
        ++usage[phase].calls;
        usage[phase].bytes += bytes;
        return ptr;
    }
    void release(void* ptr, std::size_t bytes) override {
        ++usage[phase].frees;
        usage[phase].freedBytes += bytes;
        _aligned_free(ptr);
    }
};

bool advance(Rig& rig, double forceY) {
    for (int i = 0; i < 15; ++i)
        if (!rig.step(1.0 / 60, 10, forceY) || !rig.read().valid()) return false;
    return true;
}
}

int main() {
    Counter counter;
    double travelMm;
    {
        Rig rig({0.5, 10, 10}, true, counter);
        counter.phase = preload;
        if (!advance(rig, 0)) return 2;
        const double start = rig.read().relativeY;
        counter.phase = loaded;
        if (!advance(rig, 10)) return 2;
        travelMm = (start - rig.read().relativeY) * 1000;
        counter.phase = destruction;
    }
    const char* names[] = {"construction", "preload", "loaded", "destruction"};
    std::size_t allocations = 0, frees = 0;
    std::puts("phase,scalar_bytes,frames,updates,upstream_allocations,requested_bytes,upstream_frees,released_bytes,end_drift_mm");
    for (const auto phase : {construction, preload, loaded, destruction}) {
        const auto& u = counter.usage[phase];
        const int frames = phase == preload || phase == loaded ? 15 : 0;
        std::printf("%s,%zu,%d,%d,%zu,%zu,%zu,%zu,", names[phase], sizeof(reactphysics3d::decimal),
                    frames, 8 * frames, u.calls, u.bytes, u.frees, u.freedBytes);
        if (phase == loaded) std::printf("%.12g", travelMm);
        std::putchar('\n');
        allocations += u.calls;
        frees += u.frees;
    }
    return allocations == frees && std::isfinite(travelMm) ? 0 : 1;
}
