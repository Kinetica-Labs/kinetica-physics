#include <math/StepMemory.h>
#include <dynamics/time_stepping/SoftJoint.h>

#include <algorithm>

namespace kp {
namespace {

// Определитель матрицы отклика меньше этой доли произведения её диагонали — связи зависимы, и
// вместе их не решить: они решаются по одной.
constexpr Real kIndependent = Real(1e-9);

Real component(Vector3 v, int k) { return k == 0 ? v.x : k == 1 ? v.y : v.z; }

} // namespace

SoftJoint::SoftJoint(const Bond* bonds, int count, const StepArray<SolverBody>& states, Real* impulses) : count_(count), impulses_(impulses) {
    const SolverBody &a = states[size_t(bonds[0].a)], &b = states[size_t(bonds[0].b)];
    // Матрица отклика; место строки, которой нет или которая ни на что не действует, занимает единица.
    Real k[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int i = 0; i < count; ++i) {
        bonds_[i] = bonds[i];
        const Vector3 spinA = a.spinFrom(bonds[i].angularA), spinB = b.spinFrom(bonds[i].angularB);
        const Real response = (a.inverseMass + b.inverseMass) * bonds[i].linear.dot(bonds[i].linear) + bonds[i].angularA.dot(spinA) + bonds[i].angularB.dot(spinB);
        mass_[i] = response > 0 ? 1 / response : 0;
        responds_ = responds_ || response > 0;
        if (response <= 0) continue;
        k[i][i] = response;
        for (int j = 0; j < i; ++j)
            if (mass_[j] > 0) k[i][j] = k[j][i] = (a.inverseMass + b.inverseMass) * bonds[j].linear.dot(bonds[i].linear) + bonds[j].angularA.dot(spinA) + bonds[j].angularB.dot(spinB);
    }
    if (count < 2) return;
    const Matrix3x3 matrix{{{k[0][0], k[0][1], k[0][2]}, {k[1][0], k[1][1], k[1][2]}, {k[2][0], k[2][1], k[2][2]}}};
    together_ = matrix.determinant() > kIndependent * k[0][0] * k[1][1] * k[2][2];
    if (together_) inverse_ = matrix.inverse();
}

Real SoftJoint::speed(int row, const StepArray<SolverBody>& states) const {
    const Bond& bond = bonds_[row];
    const SolverBody &a = states[size_t(bond.a)], &b = states[size_t(bond.b)];
    return bond.linear.dot(b.v - a.v) + bond.angularB.dot(b.w) - bond.angularA.dot(a.w);
}

void SoftJoint::push(int row, StepArray<SolverBody>& states, Real impulse) const {
    const Bond& bond = bonds_[row];
    SolverBody &a = states[size_t(bond.a)], &b = states[size_t(bond.b)];
    a.v -= (a.inverseMass * impulse) * bond.linear, a.w -= a.spinFrom(impulse * bond.angularA);
    b.v += (b.inverseMass * impulse) * bond.linear, b.w += b.spinFrom(impulse * bond.angularB);
}

void SoftJoint::warmStart(StepArray<SolverBody>& states) const {
    for (int i = 0; i < count_; ++i)
        if (mass_[i] > 0) push(i, states, impulses_[i]);
}

void SoftJoint::solve(StepArray<SolverBody>& states, bool withBias, Real h, const Softness& soft, Real pushSpeed) {
    if (together_) return solveTogether(states, withBias ? soft : Softness::rigid());
    for (int i = 0; i < count_; ++i)
        if (mass_[i] > 0) solveAlone(i, states, withBias, h, soft, pushSpeed);
}

// Δλ = −K⁻¹ · доля · (скорость + поправка · нарушение) − возврат · λ: то же, что у одной связи,
// с матрицей на месте числа.
void SoftJoint::solveTogether(StepArray<SolverBody>& states, const Softness& s) {
    Real wanted[3] = {0, 0, 0};
    for (int i = 0; i < count_; ++i)
        if (mass_[i] > 0) wanted[i] = s.massScale * (speed(i, states) + s.biasRate * bonds_[i].error);
    const Vector3 change = inverse_ * Vector3{wanted[0], wanted[1], wanted[2]};
    for (int i = 0; i < count_; ++i) {
        if (mass_[i] == 0) continue;
        const Real step = -component(change, i) - s.impulseScale * impulses_[i];
        impulses_[i] += step;
        push(i, states, step);
    }
}

void SoftJoint::solveAlone(int row, StepArray<SolverBody>& states, bool withBias, Real h, const Softness& soft, Real pushSpeed) {
    const Bond& bond = bonds_[row];
    const Real mass = mass_[row], speed = this->speed(row, states);
    Real& impulse = impulses_[row];
    Real change;
    if (bond.soft) { // пружина с демпфером, неявно: γ = 1 / (h² k + h c)
        const Real give = 1 / (h * h * bond.stiffness + h * bond.damping);
        change = -(speed + give * h * bond.stiffness * bond.stretch + give * impulse) / (1 / mass + give);
    } else if (bond.driven) {
        const Real limit = bond.strength * h, wanted = impulse - mass * (speed - bond.speed);
        change = std::max(-limit, std::min(limit, wanted)) - impulse;
    } else if (bond.oneSided) {
        const Softness s = withBias && bond.gap <= 0 ? soft : Softness::rigid();
        const Real bias = bond.gap > 0 ? bond.gap / h : std::max(s.massScale * s.biasRate * bond.gap, -pushSpeed);
        change = std::max(impulse - mass * (s.massScale * speed + bias) - s.impulseScale * impulse, Real(0)) - impulse;
    } else {
        const Softness s = withBias ? soft : Softness::rigid();
        change = -mass * s.massScale * (speed + s.biasRate * bond.error) - s.impulseScale * impulse;
    }
    impulse += change;
    push(row, states, change);
}

} // namespace kp
