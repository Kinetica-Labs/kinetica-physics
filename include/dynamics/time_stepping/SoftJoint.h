#pragma once
// Связи сочленения в шаге с постоянной ценой кадра: мягкая поправка нарушения на первом проходе,
// чистая скорость на втором.
//
// Запертые свободы одного рода — до трёх смещений общей точки или до трёх поворотов — решаются
// вместе, одной матрицей отклика K = J M⁻¹ Jᵀ (так же в ReactPhysics3D, Box3D и Jolt): решённые
// по одной, они сбивают друг друга и сходятся только за много проходов. Пружина, привод и предел
// хода — по одной связи.

#include <math/StepMemory.h>
#include <dynamics/time_stepping/Softness.h>
#include <dynamics/time_stepping/SolverBody.h>
#include <joints/Bond.h>
#include <math/Matrix3x3.h>

#include <vector>

namespace kp {

class SoftJoint {
public:
    // impulses — место накопленных импульсов этих связей; оно переживает кадр.
    SoftJoint(const Bond* bonds, int count, const StepArray<SolverBody>& states, Real* impulses);
    const Bond& firstRow() const { return bonds_[0]; }
    bool responds() const { return responds_; }
    void warmStart(StepArray<SolverBody>& states) const;
    void solve(StepArray<SolverBody>& states, bool withBias, Real h, const Softness& soft, Real pushSpeed);

private:
    Real speed(int row, const StepArray<SolverBody>& states) const;
    void push(int row, StepArray<SolverBody>& states, Real impulse) const;
    void solveTogether(StepArray<SolverBody>& states, const Softness& soft);
    void solveAlone(int row, StepArray<SolverBody>& states, bool withBias, Real h, const Softness& soft, Real pushSpeed);

    Bond bonds_[3];
    int count_;
    Real mass_[3];      // 1 / отклик вдоль линии; ноль — линия ни на что не действует
    Matrix3x3 inverse_; // K⁻¹ связей, решаемых вместе
    bool together_ = false, responds_ = false;
    Real* impulses_;
};

} // namespace kp
