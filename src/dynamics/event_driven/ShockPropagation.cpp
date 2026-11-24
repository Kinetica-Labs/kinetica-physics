#include <dynamics/event_driven/ShockPropagation.h>
#include "../SupportGraph.h"

namespace kp {

void ShockPropagation::project(size_t i, Real goal) {
    if (goal == ContactRows::kIdle) return;
    const auto& row = state_.rows[i];
    const Real h = state_.holdTime;
    // Сдвиг на отрезке опоры вычисляется по средней начальной и конечной скорости.
    // Зазор s разрешает ещё 2s/h сближения в конце отрезка. Цель ≤ 0 исключает отскок.
    const Real target = std::min(Real(0), goal - 2 * std::max(Real(0), row.separation) / h);
    const Real excess = target - state_.relativeSpeed(row, row.normal);
    if (Real(.5) * h * excess <= kContactDistance) return;
    const Real impulse = row.normalMass * excess;
    state_.apply(row, impulse * row.normal);
    state_.sum[i] += impulse; // Н·с, входит в сохранённую силу опоры
}

void ShockPropagation::pass(const StepArray<Real>& goal) {
    if (state_.pairs.empty() || state_.holdTime <= 0) return;
    const auto collect = [&](SupportGraph& graph) {
        for (size_t p = 0; p < state_.pairs.size(); ++p) {
            const auto& pair = state_.pairs[p];
            for (size_t i = pair.first; i < pair.last; ++i) {
                if (goal[i] != ContactRows::kIdle) {
                    graph.add(p, pair.a, pair.b, pair.normal);
                    break;
                }
            }
        }
    };
    sweepSupports(state_.bodies, -1 * state_.up, collect, [&](size_t p, bool reverse) {
        const auto& pair = state_.pairs[p];
        for (size_t k = 0; k < pair.last - pair.first; ++k) {
            const size_t i = reverse ? pair.last - 1 - k : pair.first + k;
            project(i, goal[i]);
        }
    });
}

} // namespace kp
