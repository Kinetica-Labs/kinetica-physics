#include <math/Quaternion.h>

namespace kp {

Quaternion Quaternion::fromRotationVector(Vector3 theta) {
    const Real angle = theta.length();
    const SinCos half = sinCos(angle / 2);
    const Real scale = angle < Real(1e-4) ? Real(0.5) - angle * angle / 48 : half.sin / angle;
    return {half.cos, scale * theta.x, scale * theta.y, scale * theta.z};
}

// Элементы матрицы поворота — столбцы x, y, z. След 1 + 2 cos θ; при следе меньше нуля делитель берётся от
// наибольшего диагонального элемента.
Quaternion Quaternion::fromAxes(Vector3 x, Vector3 y, Vector3 z) {
    const Real trace = x.x + y.y + z.z;
    if (trace > 0) {
        const Real s = 2 * sqrt(1 + trace);
        return {s / 4, (y.z - z.y) / s, (z.x - x.z) / s, (x.y - y.x) / s};
    }
    if (x.x > y.y && x.x > z.z) {
        const Real s = 2 * sqrt(1 + x.x - y.y - z.z);
        return {(y.z - z.y) / s, s / 4, (y.x + x.y) / s, (z.x + x.z) / s};
    }
    if (y.y > z.z) {
        const Real s = 2 * sqrt(1 + y.y - x.x - z.z);
        return {(z.x - x.z) / s, (y.x + x.y) / s, s / 4, (z.y + y.z) / s};
    }
    const Real s = 2 * sqrt(1 + z.z - x.x - y.y);
    return {(x.y - y.x) / s, (z.x + x.z) / s, (z.y + y.z) / s, s / 4};
}

} // namespace kp
