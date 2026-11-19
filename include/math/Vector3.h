#pragma once
// Вектор в трёхмерном пространстве.

#include <math/math_functions.h>

namespace kp {

struct Vector3 {
    Real x = 0, y = 0, z = 0;

    Real dot(Vector3 other) const { return x * other.x + y * other.y + z * other.z; }
    Vector3 cross(Vector3 other) const { return {y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x}; }
    Real lengthSquare() const { return dot(*this); }
    Real length() const { return sqrt(dot(*this)); }
    // Покомпонентное произведение: этот вектор как диагональная матрица, применённая к другому.
    Vector3 scaled(Vector3 other) const { return {x * other.x, y * other.y, z * other.z}; }
};

inline Vector3 operator+(Vector3 a, Vector3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vector3 operator-(Vector3 a, Vector3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vector3 operator-(Vector3 v) { return {-v.x, -v.y, -v.z}; }
inline Vector3 operator*(Real s, Vector3 v) { return {s * v.x, s * v.y, s * v.z}; }
inline Vector3& operator+=(Vector3& a, Vector3 b) { return a = a + b; }
inline Vector3& operator-=(Vector3& a, Vector3 b) { return a = a - b; }

} // namespace kp
