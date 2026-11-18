#pragma once
// Проверки без сторонних библиотек: условие, а при отказе — файл, строка и сами числа.

#include <cstdio>

namespace kp::test {

inline int failures = 0;
inline int checks = 0;

#define CHECK(condition, ...)                                                        \
    do {                                                                             \
        ++kp::test::checks;                                                          \
        if (!(condition)) {                                                          \
            ++kp::test::failures;                                                    \
            std::printf("ОТКАЗ %s:%d: %s — ", __FILE__, __LINE__, #condition);       \
            std::printf(__VA_ARGS__);                                                \
            std::printf("\n");                                                       \
        }                                                                            \
    } while (0)

// Совпадение с эталоном в пределах допуска.
#define CHECK_NEAR(value, expected, tolerance)                                                    \
    CHECK(std::fabs((value) - (expected)) <= (tolerance), "получено %.17g, ожидалось %.17g, допуск %g", \
          double(value), double(expected), double(tolerance))

} // namespace kp::test
