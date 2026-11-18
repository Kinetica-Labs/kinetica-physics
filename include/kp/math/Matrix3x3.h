#pragma once
// Матрица 3 × 3: тензор инерции тела в мире и обратный к нему.

#include <kp/math/Vector3.h>

namespace kp {

struct Matrix3x3 {
    Vector3 rows[3];

    static Matrix3x3 fromColumns(Vector3 c0, Vector3 c1, Vector3 c2) { return {{{c0.x, c1.x, c2.x}, {c0.y, c1.y, c2.y}, {c0.z, c1.z, c2.z}}}; }
    static Matrix3x3 diagonal(Vector3 d) { return {{{d.x, 0, 0}, {0, d.y, 0}, {0, 0, d.z}}}; }

    Vector3 column(int k) const;
    Matrix3x3 transposed() const { return fromColumns(rows[0], rows[1], rows[2]); }
    Real determinant() const { return rows[0].dot(rows[1].cross(rows[2])); }
    // Обратная матрица через присоединённую; вырожденной матрице обращение не положено.
    Matrix3x3 inverse() const;
};

inline Vector3 operator*(const Matrix3x3& m, Vector3 v) { return {m.rows[0].dot(v), m.rows[1].dot(v), m.rows[2].dot(v)}; }
Matrix3x3 operator*(const Matrix3x3& a, const Matrix3x3& b);

} // namespace kp
