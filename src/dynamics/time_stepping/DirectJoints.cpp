#include <kp/dynamics/time_stepping/DirectJoints.h>

#include <algorithm>

namespace kp {

void DirectJoints::clear() {
    bonds_.clear(), impulses_.clear();
    response_ = {};
}

void DirectJoints::add(const Bond& bond, Real* impulse) { bonds_.push_back(bond), impulses_.push_back(impulse); }

void DirectJoints::assemble(const std::vector<SolverBody>& states) {
    // Связи одного тела: строка i матрицы начинается с самой ранней связи, с которой у неё есть
    // общее подвижное тело.
    std::vector<std::vector<int>> onBody(states.size());
    for (size_t i = 0; i < bonds_.size(); ++i) {
        const Bond& bond = bonds_[i];
        int leftmost = int(i);
        for (int body : {bond.a, bond.b})
            if (states[size_t(body)].inverseMass > 0 && !onBody[size_t(body)].empty()) leftmost = std::min(leftmost, onBody[size_t(body)].front());
        response_.addRow(leftmost);
        for (int body : {bond.a, bond.b}) {
            const SolverBody& state = states[size_t(body)];
            if (state.inverseMass == 0) continue;
            // Единичный импульс вдоль связи i: телу b — плюс, телу a — минус.
            const Real sign = body == bond.b ? Real(1) : Real(-1);
            const Vector3 velocity = (sign * state.inverseMass) * bond.linear, spin = sign * state.spinFrom(body == bond.b ? bond.angularB : bond.angularA);
            for (int j : onBody[size_t(body)]) {
                const Bond& other = bonds_[size_t(j)];
                const bool isB = body == other.b;
                response_.at(int(i), j) += (isB ? Real(1) : Real(-1)) * (other.linear.dot(velocity) + (isB ? other.angularB : other.angularA).dot(spin));
            }
            response_.at(int(i), int(i)) += sign * (bond.linear.dot(velocity) + (body == bond.b ? bond.angularB : bond.angularA).dot(spin));
            if (body != bond.a || bond.a != bond.b) onBody[size_t(body)].push_back(int(i));
        }
    }
}

void DirectJoints::push(size_t row, std::vector<SolverBody>& states, Real impulse) const {
    const Bond& bond = bonds_[row];
    SolverBody &a = states[size_t(bond.a)], &b = states[size_t(bond.b)];
    a.v -= (a.inverseMass * impulse) * bond.linear, a.w -= a.spinFrom(impulse * bond.angularA);
    b.v += (b.inverseMass * impulse) * bond.linear, b.w += b.spinFrom(impulse * bond.angularB);
}

void DirectJoints::warmStart(std::vector<SolverBody>& states) const {
    for (size_t i = 0; i < bonds_.size(); ++i) push(i, states, *impulses_[i]);
}

void DirectJoints::solve(std::vector<SolverBody>& states, Real correction, Real h) {
    if (bonds_.empty()) return;
    std::vector<Real> change(bonds_.size());
    for (size_t i = 0; i < bonds_.size(); ++i) {
        const Bond& bond = bonds_[i];
        const SolverBody &a = states[size_t(bond.a)], &b = states[size_t(bond.b)];
        // Привод с неограниченной силой — та же связь, только скорость вдоль неё не ноль, а заданная.
        change[i] = -(bond.linear.dot(b.v - a.v) + bond.angularB.dot(b.w) - bond.angularA.dot(a.w) + (bond.driven ? -bond.speed : correction * bond.error / h));
    }
    ProfileMatrix factor = response_;
    solveCholesky(factor, change);
    for (size_t i = 0; i < bonds_.size(); ++i) *impulses_[i] += change[i], push(i, states, change[i]);
}

} // namespace kp
