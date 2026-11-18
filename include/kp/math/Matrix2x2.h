#pragma once
// Матрица 2 × 2: отклик трения пары в касательной плоскости.

#include <kp/math/Vector2.h>

namespace kp {

struct Matrix2x2 {
    Real m11 = 0, m12 = 0, m21 = 0, m22 = 0;

    Real determinant() const { return m11 * m22 - m12 * m21; }
    // Обратная матрица; у вырожденной (определитель не положителен) — нулевая: такая пара
    // трения не передаёт.
    Matrix2x2 inverse() const {
        const Real d = determinant();
        if (!(d > 0)) return {};
        return {m22 / d, -m12 / d, -m21 / d, m11 / d};
    }
};

inline Vector2 operator*(const Matrix2x2& m, Vector2 v) { return {m.m11 * v.x + m.m12 * v.y, m.m21 * v.x + m.m22 * v.y}; }

} // namespace kp
