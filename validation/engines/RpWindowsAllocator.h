#pragma once
// В Windows парный к _aligned_malloc вызов освобождения — _aligned_free.
#ifdef _WIN32
#include <malloc.h>
#include <reactphysics3d/reactphysics3d.h>

class RpWindowsAllocator final : public reactphysics3d::MemoryAllocator {
    void* allocate(size_t size) override { return _aligned_malloc(size, reactphysics3d::GLOBAL_ALIGNMENT); }
    void release(void* p, size_t) override { _aligned_free(p); }
};
#endif
