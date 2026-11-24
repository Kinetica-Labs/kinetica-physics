#pragma once
// Движение тела между ударами: силы меняют импульсы, импульсы меняют позу.

#include <body/Body.h>

#include <algorithm>

namespace kp {

// Толчок: p += F dt, L += τ dt. Сила и момент постоянны на шаге.
inline void integrateVelocity(Body& body, Vector3 force, Vector3 torque, Real dt) {
    if (body.mass.inverseMass == 0) return;
    body.linearMomentum += dt * force;
    body.angularMomentum += dt * torque;
}

// Тело без массы идёт с заданными скоростями (Body::drivenVelocity, drivenSpin): x += v dt, поворот —
// точная экспонента ω dt. Неподвижное не трогается вовсе: его поза обязана остаться прежней бит в бит.
inline void carryDriven(Body& body, Real dt) {
    if (!isDriven(body)) return;
    body.position += dt * body.drivenVelocity;
    if (body.drivenSpin.x != 0 || body.drivenSpin.y != 0 || body.drivenSpin.z != 0) body.orientation = (Quaternion::fromRotationVector(dt * body.drivenSpin) * body.orientation).unit();
}

// Наибольший поворот несимметричного тела за одну часть шага, рад.
constexpr Real kMaxTurn = Real(0.25);

// γ = 1 / (2 − ∛2) — вес крайних шагов композиции четвёртого порядка.
constexpr Real kJump = Real(1.3512071919596578);

// Поворот тела за время step по правилу средней точки: θ = step · I⁻¹ L(середина), где momentum —
// момент импульса в осях тела, inverse — обратные главные моменты; уравнение решается методом Ньютона.
inline Vector3 midpointTurn(Vector3 momentum, Vector3 inverse, Real step) {
    Vector3 turn = step * inverse.scaled(momentum);
    for (int iteration = 0; iteration < 4; ++iteration) {
        const Vector3 half = Real(-0.5) * turn;
        const Vector3 middle = Quaternion::fromRotationVector(half).rotate(momentum); // L в середине части
        const Vector3 residual = turn - step * inverse.scaled(middle);
        // Матрица Ньютона точная: приблизительная сходится у вытянутых тел слишком медленно.
        // Добавка δ к θ поворачивает L середины на −J δ / 2, где J = I + a [u]× + b [u]×² —
        // производная экспоненты в точке u = −θ/2; a = (1 − cos|u|) / |u|² и
        // b = (|u| − sin|u|) / |u|³ взяты рядами, |u| здесь не больше kMaxTurn / 2.
        const Real squared = half.dot(half), a = Real(0.5) - squared / 24, b = Real(1) / 6 - squared / 120;
        Vector3 column[3];
        const Vector3 axis[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        for (int k = 0; k < 3; ++k) {
            const Vector3 swept = axis[k] + a * half.cross(axis[k]) + b * half.cross(half.cross(axis[k]));
            column[k] = axis[k] + (step / 2) * inverse.scaled(swept.cross(middle));
        }
        // Правило Крамера для системы 3 × 3.
        const Real determinant = column[0].dot(column[1].cross(column[2]));
        turn = turn - (1 / determinant) * Vector3{residual.dot(column[1].cross(column[2])), column[0].dot(residual.cross(column[2])),
                                              column[0].dot(column[1].cross(residual))};
    }
    return turn;
}

// Свободный полёт: поза меняется при неизменных p и L.
//
// Перенос точен: x += v dt. Вращение — правило средней точки на группе вращений:
//   q₁ = exp(θ) q₀,   θ = ω(q½) h,   q½ = exp(θ / 2) q₀ — середина дуги между q₀ и q₁.
// Угловая скорость берётся в середине шага, потому что у несимметричного тела она меняется
// вместе с ориентацией. Точное решение этого уравнения симметрично во времени, поэтому
// энергия вращения не уходит в одну сторону, а колеблется около истинной; момент импульса
// не трогается вовсе. Поворот — точная экспонента: тело с равными моментами инерции, у которого
// ω постоянна, поворачивается за шаг ровно на ω dt при любой скорости вращения. О симметричных
// методах на группах вращений: Hairer, Lubich, Wanner, «Geometric Numerical Integration», главы
// IV и VII.
//
// Уравнение на θ неявное. Оно решается в осях тела, где тензор инерции диагонален: поворот
// вокруг θ саму θ не меняет, поэтому в этих осях уравнение имеет вид θ = h I⁻¹ exp(−θ/2) L₀.
// Решает его метод Ньютона с фиксированным числом итераций. Простые итерации здесь не годятся: у
// вытянутого тела, вращающегося вокруг поперечной оси, малый наклон меняет угловую скорость в
// Iпоперёк / Iвдоль раз сильнее самого наклона — у стержня в сто раз, — и они расходятся уже при
// повороте на сотую долю радиана.
//
// Быстрое вращение несимметричного тела делится на части по kMaxTurn радиан. Длина части зависит
// только от состояния тела в её начале, поэтому путь за время t — начало пути за любое большее
// время: поиск времени удара, который спрашивает позу в разные моменты, идёт по одной траектории.
inline void integratePosition(Body& body, Real dt) {
    if (body.mass.inverseMass == 0) return carryDriven(body, dt);
    body.position += dt * body.linearVelocity();
    const Vector3 inverse = body.mass.inverseInertia;
    const bool symmetric = inverse.x == inverse.y && inverse.y == inverse.z; // ω постоянна: делить нечего

    for (Real remaining = dt; remaining > 0;) {
        const Real speed = body.angularVelocity().length();
        const Real h = symmetric || speed * remaining <= kMaxTurn ? remaining : kMaxTurn / speed;
        remaining -= h;

        // Часть проходится тремя шагами средней точки длиной γh, (1 − 2γ)h, γh, γ = 1 / (2 − ∛2):
        // симметричная композиция, в которой ошибки второго порядка трёх шагов гасят друг друга,
        // и остаётся четвёртый (Yoshida, 1990; Hairer, Lubich, Wanner, раздел II.4). Средний шаг
        // идёт назад во времени — для свободного вращения это законно.
        for (Real weight : {kJump, 1 - 2 * kJump, kJump}) {
            const Vector3 momentum = body.orientation.rotateInverse(body.angularMomentum); // L₀ в осях тела
            body.orientation = (body.orientation * Quaternion::fromRotationVector(midpointTurn(momentum, inverse, weight * h))).unit();
        }
    }
}

// Тело через время t от начала отрезка при постоянных силе и моменте: половина толчка, полёт,
// вторая половина (схема Штёрмера — Верле). Она симметрична во времени, а при постоянной силе
// даёт параболу x₀ + v₀ t + F t²/2m точно. Несимметричная схема «толчок, потом полёт» теряла бы
// на каждом отскоке долю g·dt/v энергии: её сохраняемая величина меняется при смене знака скорости.
inline Body advanced(Body body, Vector3 force, Vector3 torque, Real t) {
    integrateVelocity(body, force, torque, t / 2);
    integratePosition(body, t);
    integrateVelocity(body, force, torque, t / 2);
    return body;
}

} // namespace kp
