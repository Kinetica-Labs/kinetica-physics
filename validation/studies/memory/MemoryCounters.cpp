// Подмена new действует только в отдельной программе для подсчёта, в замере времени её нет.
#include "MemoryCounters.h"
#include <algorithm>
#include <cstdlib>
#include <new>
#ifdef _WIN32
#include <malloc.h>
#endif

namespace memory_study {
namespace {
thread_local Counts counts;
thread_local bool enabled = false;
}
void start() { counts = {}; enabled = true; }
Counts stop() { enabled = false; return counts; }
void heap(std::size_t bytes) {
    if (enabled) { ++counts.heapCalls; counts.heapBytes += bytes; }
}
void refill(std::size_t bytes) {
    if (enabled) { ++counts.refills; counts.refillBytes += bytes; }
}
void issue(std::size_t bytes, std::size_t occupied, std::size_t held, std::size_t blocks) {
    if (!enabled) return;
    ++counts.arenaCalls;
    counts.payload += bytes;
    counts.occupied += occupied;
    counts.peak = std::max(counts.peak, held);
    counts.blocks = std::max(counts.blocks, blocks);
}
}

namespace {
void* allocate(std::size_t bytes) {
    if (void* p = std::malloc(bytes ? bytes : 1)) { memory_study::heap(bytes); return p; }
    throw std::bad_alloc();
}
void* aligned(std::size_t bytes, std::size_t alignment) {
#ifdef _WIN32
    void* p = _aligned_malloc(bytes ? bytes : 1, alignment);
#else
    void* p = nullptr;
    if (posix_memalign(&p, alignment, bytes ? bytes : 1) != 0) p = nullptr;
#endif
    if (!p) throw std::bad_alloc();
    memory_study::heap(bytes);
    return p;
}
void releaseAligned(void* p) {
#ifdef _WIN32
    _aligned_free(p);
#else
    std::free(p);
#endif
}
}
void* operator new(std::size_t n) { return allocate(n); }
void* operator new[](std::size_t n) { return allocate(n); }
void* operator new(std::size_t n, std::align_val_t a) { return aligned(n, std::size_t(a)); }
void* operator new[](std::size_t n, std::align_val_t a) { return aligned(n, std::size_t(a)); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void operator delete(void* p, std::align_val_t) noexcept { releaseAligned(p); }
void operator delete[](void* p, std::align_val_t) noexcept { releaseAligned(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { releaseAligned(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { releaseAligned(p); }
