#pragma once
// Память на время шага мира.
//
// Решатель создаёт за шаг десятки тысяч рабочих массивов — строки систем, матрицы, списки точек, — и
// все они не нужны после шага. Пока в потоке открыт StepMemory, такие массивы (StepArray) берут
// память сдвигом указателя из запаса потока, а не из кучи; освобождения нет — запас целиком
// возвращается, когда закрывается самый внешний StepMemory. Вне шага StepArray работает как обычный
// std::vector, через кучу: приборы и проверки, вызывающие части решателя напрямую, ничего не замечают.
//
// Правило одно: StepArray не должен пережить шаг. Всё, что живёт между шагами — тела, память сил
// контакта и импульсов сочленений, рабочие массивы потоков, — хранится в std::vector.
// С переменной окружения KP_STEP_MEMORY=check запас при возврате затирается: массив, переживший шаг,
// даёт мусор сразу, а не когда-нибудь (так проверяются правки).

#include <cstddef>
#include <vector>

namespace kp {

class StepMemory {
public:
    StepMemory();
    ~StepMemory();
    StepMemory(const StepMemory&) = delete;
    StepMemory& operator=(const StepMemory&) = delete;

    // Память под bytes байт, выровненная на 16; освобождается через give.
    static void* take(size_t bytes);
    static void give(void* memory) noexcept;
};

template <class T> struct StepAllocator {
    using value_type = T;
    StepAllocator() = default;
    template <class U> StepAllocator(const StepAllocator<U>&) {}
    T* allocate(size_t count) { return static_cast<T*>(StepMemory::take(count * sizeof(T))); }
    void deallocate(T* memory, size_t) noexcept { StepMemory::give(memory); }
    template <class U> bool operator==(const StepAllocator<U>&) const { return true; }
    template <class U> bool operator!=(const StepAllocator<U>&) const { return false; }
};

// Массив, живущий не дольше шага мира.
template <class T> using StepArray = std::vector<T, StepAllocator<T>>;

} // namespace kp
