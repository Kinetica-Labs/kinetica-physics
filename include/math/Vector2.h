#pragma once
// Вектор на плоскости: скольжение и трение в касательной плоскости контакта.

#include <math/math_functions.h>

namespace kp {

struct Vector2 {
    Real x = 0, y = 0;

    Real dot(Vector2 other) const { return x * other.x + y * other.y; }
    Real lengthSquare() const { return dot(*this); }
    Real length() const { return sqrt(dot(*this)); }
};

inline Vector2 operator+(Vector2 a, Vector2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vector2 operator-(Vector2 a, Vector2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vector2 operator*(Real s, Vector2 v) { return {s * v.x, s * v.y}; }

} // namespace kp
