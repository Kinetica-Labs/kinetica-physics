#include <math/StepMemory.h>
#include <dynamics/event_driven/CoupledSolve.h>

#include <dynamics/event_driven/ActiveSet.h>

#include <algorithm>
#include <array>
#include <limits>

namespace kp {
namespace {

constexpr int kMaxDriveFlips = 4; // сколько раз привод может упереться в предел силы и отпуститься

} // namespace

// Накопленные импульсы и цели скорости по всем линиям системы: нормали, трение пар, связи. Пара,
// которая не скользит, за отрезок не должна сдвинуться. Без сил её точка касания сама ушла бы с
// ускорением drift, поэтому трение обязано оставить ей встречную скорость.
void CoupledSolve::startTogether(const System& s, const StepArray<Real>& goal, StepArray<Real>& current, StepArray<Real>& target) const {
    const size_t n = s.rows.size(), firstBond = n + 3 * s.pairs.size();
    for (size_t i = 0; i < n; ++i) current[i] = state_.sum[s.rows[i]], target[i] = goal[s.rows[i]];
    for (size_t p = 0; p < s.pairs.size(); ++p) {
        for (size_t k = 0; k < 3; ++k) {
            current[n + 3 * p + k] = state_.pairs[s.pairs[p]].impulse[k];
            target[n + 3 * p + k] = -state_.holdTime * state_.pairs[s.pairs[p]].drift[k];
        }
    }
    for (size_t j = 0; j < state_.bonds.size(); ++j) current[firstBond + j] = state_.bondImpulse[j], target[firstBond + j] = state_.bondGoal[j];
}

// Неизвестные: все нормали, линии трения несорвавшихся пар и связи без упора. Вращение вокруг
// нормали — только если у пары больше одной точки касания.
StepArray<char> CoupledSolve::unknownsOf(const System& s) const {
    const size_t n = s.rows.size(), firstBond = n + 3 * s.pairs.size();
    StepArray<char> unknown(s.lines.size(), 0);
    for (size_t i = 0; i < n; ++i) unknown[i] = 1;
    for (size_t p = 0; p < s.pairs.size(); ++p)
        for (size_t k = 0; k < 3; ++k) unknown[n + 3 * p + k] = state_.stuck[s.pairs[p]] && (k < 2 || state_.pairs[s.pairs[p]].reach > 0);
    for (size_t j = 0; j < state_.bonds.size(); ++j) unknown[firstBond + j] = state_.bondActive[j] && !state_.saturated[j];
    return unknown;
}

// Запас скорости над целью вдоль линии i при импульсах λ равен free[i] + Σ A[i][j] λ[j]; free —
// запас, какой был бы без неизвестных импульсов вовсе. Слагаемое, удерживающее импульсы вблизи
// прежних: к диагонали добавляется ε Aᵢᵢ, к свободному члену — −ε Aᵢᵢ λᵢ.
StepArray<Real> CoupledSolve::freeTerms(const System& s, const StepArray<char>& unknown, const StepArray<Real>& current, const StepArray<Real>& target) const {
    StepArray<Real> free(s.lines.size(), 0);
    for (size_t i = 0; i < s.lines.size(); ++i) {
        if (!unknown[i]) continue;
        free[i] = state_.speed(s.lines[i]) - target[i] - kProximal * s.diagonal[i] * current[i];
        for (const Entry& entry : s.matrix[i])
            if (unknown[entry.column]) free[i] -= entry.value * current[entry.column];
    }
    return free;
}

// Предел Кулона для несорвавшихся пар: сила — μ Σ λn, момент — μ Σ λn rᵢ. Пара, вышедшая за
// предел, срывается; true — сорвалась хоть одна.
bool CoupledSolve::releaseSlipping(const System& s, const StepArray<Real>& force) {
    const size_t n = s.rows.size();
    bool released = false;
    for (size_t p = 0; p < s.pairs.size(); ++p) {
        const Pair& pair = state_.pairs[s.pairs[p]];
        if (!state_.stuck[s.pairs[p]]) continue;
        Real load = 0, leverLoad = 0;
        for (size_t row = pair.first; row < pair.last; ++row) {
            if (s.rowPlace[row] == n) continue;
            load += std::max(Real(0), force[s.rowPlace[row]]);
            leverLoad += std::max(Real(0), force[s.rowPlace[row]]) * (state_.rows[row].point - pair.centre).length();
        }
        const Real* x = &force[n + 3 * p];
        const Real slack = 1 + Real(1e-12);
        if (sqrt(x[0] * x[0] + x[1] * x[1]) > pair.friction * load * slack || abs(x[2]) > pair.friction * leverLoad * slack)
            state_.stuck[s.pairs[p]] = 0, released = true;
    }
    return released;
}

// Предел силы привода. Привод, которому для заданной скорости нужен импульс больше предельного,
// прикладывает предельный и из неизвестных выходит. Упёршийся привод возвращается, если скорость
// перешла заданную: значит, полная сила уже не нужна. true — набор неизвестных изменился.
bool CoupledSolve::releaseDrives(const System& s, const StepArray<char>& unknown, StepArray<Real>& current, const StepArray<Real>& target, const StepArray<Real>& force) {
    const size_t firstBond = s.rows.size() + 3 * s.pairs.size();
    bool released = false;
    for (size_t j = 0; j < state_.bonds.size(); ++j) {
        const Real cap = state_.driveCap(j);
        const size_t i = firstBond + j;
        if (!state_.bondActive[j] || cap == std::numeric_limits<Real>::infinity()) continue;
        if (!state_.saturated[j]) {
            if (abs(force[i]) <= cap * (1 + Real(1e-12))) continue;
            state_.saturated[j] = force[i] > 0 ? 1 : -1;
            state_.applyLine(s.lines[i], state_.saturated[j] * cap - state_.bondImpulse[j]);
            current[i] = state_.bondImpulse[j] = state_.saturated[j] * cap;
            released = true;
            continue;
        }
        Real excess = state_.speed(s.lines[i]) - target[i];
        for (const Entry& entry : s.matrix[i])
            if (unknown[entry.column]) excess += entry.value * (force[entry.column] - current[entry.column]);
        if (state_.saturated[j] * excess > kSolved && state_.bondFlips[j]++ < kMaxDriveFlips) state_.saturated[j] = 0, released = true;
    }
    return released;
}

// Найденные импульсы прикладываются к телам; возвращает наибольшую поправку скорости.
Real CoupledSolve::applyTogether(const System& s, const StepArray<Real>& force) {
    const size_t n = s.rows.size(), firstBond = n + 3 * s.pairs.size();
    Real worst = 0;
    for (size_t i = 0; i < n; ++i) {
        const Row& row = state_.rows[s.rows[i]];
        const Real normal = std::max(Real(0), force[i]);
        state_.apply(row, (normal - state_.sum[s.rows[i]]) * row.normal);
        worst = std::max(worst, abs(normal - state_.sum[s.rows[i]]) / row.normalMass);
        state_.sum[s.rows[i]] = normal;
    }
    for (size_t p = 0; p < s.pairs.size(); ++p) {
        Pair& pair = state_.pairs[s.pairs[p]];
        if (!state_.stuck[s.pairs[p]]) continue;
        const Slip x = {force[n + 3 * p], force[n + 3 * p + 1], pair.reach > 0 ? force[n + 3 * p + 2] : Real(0)};
        const Slip change = {x[0] - pair.impulse[0], x[1] - pair.impulse[1], x[2] - pair.impulse[2]};
        friction_.applyFriction(pair, change, true, true);
        pair.impulse = x;
        worst = std::max({worst, abs(change[0]) * pair.response[0], abs(change[1]) * pair.response[1],
                          abs(change[2]) * pair.response[2] * pair.reach});
    }
    for (size_t j = 0; j < state_.bonds.size(); ++j) {
        if (!state_.bondActive[j] || state_.saturated[j]) continue;
        const Real impulse = state_.bonds[j].oneSided ? std::max(Real(0), force[firstBond + j]) : force[firstBond + j];
        const Real change = impulse - state_.bondImpulse[j];
        state_.applyLine(s.lines[firstBond + j], change);
        state_.bondImpulse[j] = impulse;
        worst = std::max(worst, abs(change) * s.diagonal[firstBond + j]);
    }
    return worst;
}

// Точное решение сил опоры вместе с трением покоя. Неизвестные — нормальные импульсы и импульсы
// трения тех пар, которые не скользят: у них скольжение обязано остаться нулевым, это линейные
// равенства, и они решаются одной системой с нормалями. Пара, у которой нужное трение вышло за
// предел Кулона, срывается: её трение дальше считается отдельно (solveFriction). Перебор повторяется,
// пока срываются пары или упираются приводы. Возвращает наибольшую поправку скорости.
Real CoupledSolve::solveTogether(const System& s, const StepArray<Real>& goal) {
    const size_t total = s.lines.size();
    StepArray<Real> current(total), target(total, 0), force(total);
    startTogether(s, goal, current, target);
    for (bool released = true; released;) {
        const StepArray<char> unknown = unknownsOf(s);
        ActiveSet(state_.bodies, s, state_.bonds, state_.pressing, state_.bondPressing).solve(unknown, freeTerms(s, unknown, current, target), force);
        released = releaseSlipping(s, force);
        if (releaseDrives(s, unknown, current, target, force)) released = true;
    }
    return applyTogether(s, force);
}

// Невязка точной части задачи при нынешних импульсах, в единицах скорости: насколько скорость
// давящей точки отличается от цели, а недавящей — меньше её; насколько скользит пара, которую
// держит трение покоя; насколько не выполнены связи сочленений.
Real CoupledSolve::residual(const System& s, const StepArray<Real>& goal) const {
    Real worst = 0;
    for (size_t i : s.rows) {
        const Real excess = state_.relativeSpeed(state_.rows[i], state_.rows[i].normal) - goal[i];
        worst = std::max(worst, state_.sum[i] > 0 ? abs(excess) : -excess);
    }
    for (size_t q : s.pairs) {
        if (!state_.stuck[q]) continue;
        const Slip now = friction_.slip(state_.pairs[q]);
        for (size_t k = 0; k < 3; ++k) worst = std::max(worst, abs(now[k] + state_.holdTime * state_.pairs[q].drift[k]) * (k == 2 ? state_.pairs[q].reach : Real(1)));
    }
    const size_t firstBond = s.lines.size() - state_.bonds.size();
    for (size_t j = 0; j < state_.bonds.size(); ++j) {
        if (!state_.bondActive[j] || state_.saturated[j]) continue;
        // У податливой линии цель — для скорости вместе с её собственной подачей.
        const Real excess = state_.speed(s.lines[firstBond + j]) + s.soft[firstBond + j] * state_.bondImpulse[j] - state_.bondGoal[j];
        worst = std::max(worst, state_.bonds[j].oneSided && state_.bondImpulse[j] <= 0 ? -excess : abs(excess));
    }
    return worst;
}

} // namespace kp
