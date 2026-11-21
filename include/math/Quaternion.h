#pragma once
// Единичный кватернион как ориентация тела: v_мир = q v_тело q*.

#include <math/Vector3.h>

namespace kp {

struct Quaternion {
    Real w = 1, x = 0, y = 0, z = 0;

    Quaternion conjugate() const { return {w, -x, -y, -z}; }
    Quaternion unit() const {
        const Real n = sqrt(w * w + x * x + y * y + z * z);
        return {w / n, x / n, y / n, z / n};
    }
    // q v q* в развёрнутом виде: v + 2w (u × v) + 2 u × (u × v), где u = (x, y, z).
    Vector3 rotate(Vector3 v) const {
        const Vector3 u{x, y, z};
        const Vector3 t = 2.0 * u.cross(v);
        return v + w * t + u.cross(t);
    }
    Vector3 rotateInverse(Vector3 v) const { return conjugate().rotate(v); }

    // Поворот на вектор θ: вокруг оси θ / |θ| на угол |θ| — экспонента. Кватернион равен
    // (cos |θ|/2, θ · sin(|θ|/2) / |θ|). При малом угле множитель при θ — это 0/0; там он берётся из
    // ряда sin(a/2) / a = 1/2 − a²/48 + …, следующий член которого меньше точности числа.
    // Тело, которое вращается с постоянной угловой скоростью ω, за время t поворачивается ровно на ω t.
    static Quaternion fromRotationVector(Vector3 theta);
    // Поворот, который переводит оси x, y, z в данную правую тройку единичных взаимно перпендикулярных
    // векторов. Из четырёх формул берётся та, где делитель наибольший (Shepperd, «Quaternion from rotation
    // matrix», J. Guidance and Control 1, 1978); знак кватерниона при этом не выбирается: q и −q — один поворот.
    static Quaternion fromAxes(Vector3 x, Vector3 y, Vector3 z);
    // Отображение Кэли: вектор поворота θ -> кватернион (1, θ/2) / |(1, θ/2)| — поворот на угол
    // 2 arctg(|θ|/2) = |θ| − |θ|³/12 + …. Интегратор им больше не пользуется: при вращении он отстаёт
    // по фазе (docs/DEVELOPMENT.md, «Отображение поворота»). Осталось как дешёвый способ задать
    // поворот в проверках.
    static Quaternion cayley(Vector3 theta) { return Quaternion{1, 0.5 * theta.x, 0.5 * theta.y, 0.5 * theta.z}.unit(); }
};

inline Quaternion operator*(Quaternion a, Quaternion b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

} // namespace kp
