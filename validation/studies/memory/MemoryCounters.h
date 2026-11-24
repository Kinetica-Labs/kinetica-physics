#pragma once
// Счётчики одного измеряемого потока. Запись результата выполняется после отключения счётчиков.
#include <cstddef>

namespace memory_study {
struct Counts {
    std::size_t heapCalls = 0, heapBytes = 0;
    std::size_t arenaCalls = 0, payload = 0, occupied = 0;
    std::size_t refills = 0, refillBytes = 0, peak = 0, blocks = 0;
};
void start();
Counts stop();
void issue(std::size_t bytes, std::size_t occupied, std::size_t held, std::size_t blocks);
void refill(std::size_t bytes);
}
