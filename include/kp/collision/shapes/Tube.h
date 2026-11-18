#pragma once
// Трубка: капсула или цилиндр, обращённый к другому телу боком.

#include <kp/math/Vector3.h>

namespace kp {

// Трубка: капсула или цилиндр, обращённый к другому телу боком.
struct Tube {
    Vector3 centre, axis;               // середина и направление оси
    Real halfLength = 0, radius = 0; // половина длины оси; радиус вместе со скруглением
};

} // namespace kp
