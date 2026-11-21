#include <math/StepMemory.h>
#include <dynamics/event_driven/SlidingStopSearch.h>

#include <algorithm>
#include <array>
#include <limits>

namespace kp {
namespace {

constexpr Real kSliding = ContactRows::kSliding;
constexpr Real kHalted = Real(0.05); // скольжение упало до этой доли начального — пара остановилась, см. SlidingStop

} // namespace

void SlidingStopSearch::settle(SlidingStop stopped) {
    if (!stopped.halted) return;
    for (Pair& pair : state_.pairs)
        if (pair.a == stopped.a && pair.b == stopped.b) pair.startSlip = {};
}

SlidingStop SlidingStopSearch::slidingStop(Real h, const StepArray<std::pair<int, int>>& already) const {
    SlidingStop earliest{h};
    for (size_t p = 0; p < state_.pairs.size(); ++p) {
        const Pair& pair = state_.pairs[p];
        // Прилипшая пара не скользит по определению (ContactAssembly::addFriction): её остаток скольжения
        // в начале отрезка — не скольжение, и разворота у него нет.
        if (p < state_.stuck.size() && state_.stuck[p]) continue;
        if (std::find(already.begin(), already.end(), std::make_pair(pair.a, pair.b)) != already.end()) continue;
        bool touching = false;
        for (size_t i = pair.first; i < pair.last; ++i) touching = touching || state_.rows[i].touching;
        if (!touching || pair.friction == 0) continue;
        // Скольжение s(τ) = s₀ + τ (s₁ − s₀), τ от 0 до 1; вращение вокруг нормали переведено в
        // скорость на расстоянии reach от центра. Оно развернулось, если s₀ · s₁ < 0; ближе всего
        // к нулю оно при τ = s₀ · (s₀ − s₁) / |s₀ − s₁|².
        const Slip now = friction_.slip(pair);
        const Real start[3] = {pair.startSlip[0], pair.startSlip[1], pair.reach * pair.startSlip[2]};
        const Real end[3] = {now[0], now[1], pair.reach * now[2]};
        Real speed = 0, along = 0, changed = 0, projected = 0;
        for (int k = 0; k < 3; ++k) {
            speed += start[k] * start[k];
            along += start[k] * end[k];
            changed += (start[k] - end[k]) * (start[k] - end[k]);
            projected += start[k] * (start[k] - end[k]);
        }
        if (speed < kSliding * kSliding || along >= 0) continue;
        // Квадрат наименьшего скольжения на отрезке: остановка — если оно мало рядом с начальным.
        const Real least = speed - projected * projected / changed;
        // Остановка в самом конце отрезка событием не является: отрезок и так кончается на ней.
        if (projected >= changed * (1 - Real(1e-9))) continue;
        if (h * projected / changed < earliest.time) earliest = {h * projected / changed, pair.a, pair.b, least <= kHalted * kHalted * speed};
    }
    return earliest;
}

} // namespace kp
