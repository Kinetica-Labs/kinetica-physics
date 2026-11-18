#pragma once
// Связи сочленения в шаге с постоянной ценой кадра: мягкая поправка нарушения на первом проходе,
// чистая скорость на втором.
//
// Запертые свободы одного рода — до трёх смещений общей точки или до трёх поворотов — решаются
// вместе, одной матрицей отклика K = J M⁻¹ Jᵀ (так же в ReactPhysics3D, Box3D и Jolt): решённые
// по одной, они сбивают друг друга и сходятся только за много проходов. Пружина, привод и предел
// хода — по одной связи.

#include <kp/dynamics/time_stepping/Softness.h>
#include <kp/dynamics/time_stepping/SolverBody.h>
#include <kp/joints/Bond.h>
#include <kp/math/Matrix3x3.h>

#include <vector>

namespace kp {

class SoftJoint {
public:
    // Сколько связей, начиная с bonds[0], решаются вместе: запертые свободы одного рода подряд.
    static int blockSize(const Bond* bonds, int available);
    // impulses — место накопленных импульсов этих связей; оно переживает кадр.
    SoftJoint(const Bond* bonds, int count, const std::vector<SolverBody>& states, Real* impulses);
    bool responds() const { return responds_; }
    void warmStart(std::vector<SolverBody>& states) const;
    void solve(std::vector<SolverBody>& states, bool withBias, Real h, const Softness& soft, Real pushSpeed);

private:
    Real speed(int row, const std::vector<SolverBody>& states) const;
    void push(int row, std::vector<SolverBody>& states, Real impulse) const;
    void solveTogether(std::vector<SolverBody>& states, const Softness& soft);
    void solveAlone(int row, std::vector<SolverBody>& states, bool withBias, Real h, const Softness& soft, Real pushSpeed);

    Bond bonds_[3];
    int count_;
    Real mass_[3];      // 1 / отклик вдоль линии; ноль — линия ни на что не действует
    Matrix3x3 inverse_; // K⁻¹ связей, решаемых вместе
    bool together_ = false, responds_ = false;
    Real* impulses_;
};

} // namespace kp
