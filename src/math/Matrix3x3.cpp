#include <kp/math/Matrix3x3.h>

namespace kp {

Vector3 Matrix3x3::column(int k) const {
    return k == 0 ? Vector3{rows[0].x, rows[1].x, rows[2].x} : k == 1 ? Vector3{rows[0].y, rows[1].y, rows[2].y} : Vector3{rows[0].z, rows[1].z, rows[2].z};
}

// Строки обратной матрицы — векторные произведения столбцов исходной, делённые на определитель.
Matrix3x3 Matrix3x3::inverse() const {
    const Vector3 c0 = column(0), c1 = column(1), c2 = column(2);
    const Real scale = 1 / c0.dot(c1.cross(c2));
    return {{scale * c1.cross(c2), scale * c2.cross(c0), scale * c0.cross(c1)}};
}

Matrix3x3 operator*(const Matrix3x3& a, const Matrix3x3& b) { return Matrix3x3::fromColumns(a * b.column(0), a * b.column(1), a * b.column(2)); }

} // namespace kp
