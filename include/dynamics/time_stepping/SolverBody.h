#pragma once
// Тело в решателе с проходами: скорости и обратная инерция в мире. У неподвижного всё нулевое, и
// импульсы прикладываются к нему теми же формулами без проверок.

#include <math/Matrix3x3.h>
#include <math/Quaternion.h>

namespace kp {

// Смещение центра масс и поворот тела за время: у SolverBody — с начала кадра, у контакта — с
// момента его создания.
struct BodyMotion {
    Vector3 moved;
    Quaternion turned;
};

struct SolverBody {
    Vector3 v, w;
    Matrix3x3 inverseInertia; // обратный тензор инерции в мире
    Real inverseMass = 0;
    Vector3 moved;     // смещение с начала кадра
    Quaternion turned; // поворот с начала кадра

    Vector3 spinFrom(Vector3 torque) const { return inverseInertia * torque; }
    // Импульс P в точках с плечами rA и rB: телу b — плюс, телу a — минус. Неподвижное тело не
    // трогается: его скорости и так ноль, а общее на все островки тело не должны писать разные потоки.
    static void push(SolverBody& a, SolverBody& b, Vector3 rA, Vector3 rB, Vector3 impulse) {
        if (a.inverseMass != 0) a.v -= a.inverseMass * impulse, a.w -= a.spinFrom(rA.cross(impulse));
        if (b.inverseMass != 0) b.v += b.inverseMass * impulse, b.w += b.spinFrom(rB.cross(impulse));
    }
    static void twist(SolverBody& a, SolverBody& b, Vector3 torque) { a.w -= a.spinFrom(torque), b.w += b.spinFrom(torque); }
    static Real inverseMassAlong(const SolverBody& a, const SolverBody& b, Vector3 rA, Vector3 rB, Vector3 d) {
        const Vector3 armA = rA.cross(d), armB = rB.cross(d);
        return a.inverseMass + b.inverseMass + armA.dot(a.spinFrom(armA)) + armB.dot(b.spinFrom(armB));
    }
};

} // namespace kp
