// Однопоточный прибор для release-библиотек Bullet. Считает обращения к btAlignedAlloc
// и btAlignedFree. В байтах — размер запроса Bullet до выравнивания и накладных расходов.
// Прямые malloc/new, выделения CRT, страницы ОС и операции внутри своих пулов сюда не входят.
#define main bullet_grip_main
#include "../../engines/grip_bullet.cpp"
#undef main

#include <LinearMath/btAlignedAllocator.h>
#include <cstddef>
#include <cstdint>

namespace {
struct Totals {
    unsigned long long allocs{}, bytes{}, frees{};
};

Totals counted;
bool aligned = true;
void countFree(void* memory);

// Штатная реализация Bullet зависит от его сборки. Передаём запрос ей без подмены
// выравнивания, padding и обнуления. Глобальные hooks переключаются только в этом
// отдельном процессе с однопоточным btSequentialImpulseConstraintSolver.
void* countAlloc(std::size_t size, int alignment) {
    btAlignedAllocSetCustomAligned(nullptr, nullptr);
    void* memory = btAlignedAlloc(size, alignment);
    btAlignedAllocSetCustomAligned(countAlloc, countFree);
    ++counted.allocs;
    counted.bytes += size;
    if (!memory || alignment <= 0 || reinterpret_cast<std::uintptr_t>(memory) % alignment != 0)
        aligned = false;
    return memory;
}

void countFree(void* memory) {
    btAlignedAllocSetCustomAligned(nullptr, nullptr);
    btAlignedFree(memory);
    btAlignedAllocSetCustomAligned(countAlloc, countFree);
    if (memory) ++counted.frees;
}

void row(const char* phase, int frames, Totals before, bool complete) {
    std::printf("%s,%d,%d,%llu,%llu,%llu,%llu,%llu,%llu,%d\n",
                phase, frames, 8 * frames, counted.allocs - before.allocs,
                counted.bytes - before.bytes, counted.frees - before.frees,
                counted.allocs, counted.bytes, counted.frees, int(complete));
}

bool advance(Rig& rig, const char* phase, double load) {
    const Totals before = counted;
    int frames = 0;
    while (frames < 15 && rig.step(1.0 / 60, 10, load)) ++frames;
    const bool complete = frames == 15 && rig.read().valid() && aligned;
    row(phase, frames, before, complete);
    return complete;
}
}

int main() {
    std::puts("phase,completed_frames,public_steps_in_completed_frames,alloc_calls,requested_bytes,"
              "free_calls,total_alloc_calls,total_requested_bytes,total_free_calls,complete");
    btAlignedAllocSetCustomAligned(countAlloc, countFree);
    Totals beforeDestruction;
    bool ok;
    {
        Rig rig({0.5, 10, 10});
        ok = aligned && counted.allocs > 0 && rig.read().valid();
        row("construct", 0, {}, ok);
        if (ok) ok = advance(rig, "preload", 0);
        if (ok) ok = advance(rig, "loaded", 10);
        beforeDestruction = counted;
    }
    btAlignedAllocSetCustomAligned(nullptr, nullptr);
    row("teardown", 0, beforeDestruction, ok && aligned);
    return ok && aligned ? 0 : 1;
}
