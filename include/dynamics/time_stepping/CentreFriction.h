#pragma once
// Трение пары тел в центре площадки контакта: сила в касательной плоскости в пределах μ Σ λ и момент
// вокруг нормали в пределах μ Σ λ r (r — плечо точки до центра), затем трение качения
// (RollingFriction). Сила одна на пару, а не по точкам — так же у Box3D, Jolt и ReactPhysics3D:
// накопленный импульс переживает смену состава точек.
#include <dynamics/time_stepping/RollingFriction.h>
#include <dynamics/time_stepping/SolverBody.h>
#include <math/Matrix2x2.h>

namespace kp {

class CentreFriction {
public:
    // Оси контакта и плечи его центра от центров масс тел — нынешние.
    struct Frame {
        Vector3 normal, tangent1, tangent2, centreA, centreB;
    };
    // Накопленное с прошлого шага: импульс трения, мир, и момент вокруг нормали.
    struct Carried {
        Vector3 rubbed;
        Real twist = 0;
    };
    Real coefficient = 0; // μ пары
    // Отклик тел в центре площадки: обратная матрица 2 × 2 в касательной плоскости и обратный отклик
    // вращения вокруг нормали.
    void prepare(const SolverBody& a, const SolverBody& b, const Frame& at, const Carried& carried);
    void warmStart(SolverBody& a, SolverBody& b, const Frame& at) const;
    // load — сумма нормальных импульсов точек, leverLoad — сумма их произведений на плечо до центра.
    void solve(SolverBody& a, SolverBody& b, const Frame& at, Real load, Real leverLoad, Real rollingArm);
    Carried carried(const Frame& at) const { return {rub_.x * at.tangent1 + rub_.y * at.tangent2, twistImpulse_}; }

private:
    Matrix2x2 tangentMass_; // обратная матрица отклика трения в центре
    Real twistMass_ = 0;
    Vector2 rub_; // накопленный импульс трения вдоль двух касательных
    Real twistImpulse_ = 0;
    RollingFriction rolling_;
};

} // namespace kp
