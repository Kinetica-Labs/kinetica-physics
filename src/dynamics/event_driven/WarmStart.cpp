#include <dynamics/event_driven/WarmStart.h>

#include <algorithm>
#include <array>
#include <limits>

namespace kp {
namespace {

constexpr Real kIdle = ContactRows::kIdle;

} // namespace

// Прошлые силы, приложенные к нынешним скоростям. Изменение кинетической энергии от импульсов
// α λ равно α c + α² q / 2, где c = λ · (J v), q = λ · A λ. Наименьшее оно при α = −c / q; больше
// единицы масштаб не берётся — прошлые силы не преувеличиваются.
void WarmStart::apply(const std::vector<Real>& goal, const ContactForces& previous, Real h) {
    Real c = 0;
    for (Pair& pair : state_.pairs) {
        for (size_t i = pair.first; i < pair.last; ++i) {
            const auto found = previous.find(state_.rows[i].key);
            if (goal[i] == kIdle || !state_.rows[i].touching || found == previous.end()) continue;
            state_.sum[i] = h * found->second.normal;
            c += state_.sum[i] * state_.relativeSpeed(state_.rows[i], state_.rows[i].normal);
            pair.impulse[0] += h * found->second.tangent.dot(pair.tangent1);
            pair.impulse[1] += h * found->second.tangent.dot(pair.tangent2);
            pair.impulse[2] += h * found->second.twist;
        }
        const Slip now = friction_.slip(pair);
        for (size_t k = 0; k < 3; ++k) c += pair.impulse[k] * now[k];
    }
    const auto applyScaled = [&](Real scale) {
        for (size_t i = 0; i < state_.rows.size(); ++i) state_.apply(state_.rows[i], (scale * state_.sum[i]) * state_.rows[i].normal);
        for (const Pair& pair : state_.pairs) friction_.applyFriction(pair, {scale * pair.impulse[0], scale * pair.impulse[1], scale * pair.impulse[2]}, true, true);
    };
    const Real before = state_.kineticEnergy();
    applyScaled(1);
    const Real q = 2 * (state_.kineticEnergy() - before - c);
    const Real alpha = q > 0 ? std::min(Real(1), std::max(Real(0), -c / q)) : Real(c <= 0);
    if (alpha < 1) {
        applyScaled(alpha - 1);
        for (Real& normal : state_.sum) normal *= alpha;
        for (Pair& pair : state_.pairs) pair.impulse = {alpha * pair.impulse[0], alpha * pair.impulse[1], alpha * pair.impulse[2]};
    }
}

} // namespace kp
