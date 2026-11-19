#include <math/StepMemory.h>
#include <dynamics/event_driven/ShockPropagation.h>

#include <algorithm>
#include <array>
#include <limits>

namespace kp {
namespace {

constexpr Real kIdle = ContactRows::kIdle;

} // namespace

void ShockPropagation::pass(const StepArray<Real>& goal) {
    const Body frozen; // неподвижное тело: обратные масса и инерция нулевые
    for (Pair& pair : state_.pairs) { // пары уже упорядочены снизу вверх
        Body& a = state_.bodies[size_t(pair.a)];
        Body& b = state_.bodies[size_t(pair.b)];
        const bool upperIsB = state_.up.dot(b.position - a.position) > 0;
        Body& upper = upperIsB ? b : a;
        if (upper.isStatic()) continue;
        const Real sign = upperIsB ? Real(1) : Real(-1); // импульс +J получает тело b, −J — тело a
        bool active = false;
        for (size_t i = pair.first; i < pair.last; ++i) {
            if (goal[i] == kIdle) continue;
            active = true;
            const Row& row = state_.rows[i];
            const Real approaching = goal[i] - state_.relativeSpeed(row, row.normal);
            if (approaching > 0) applyImpulse(upper, (sign * approaching * ContactRows::effectiveMass(frozen, upper, row.point, row.normal)) * row.normal, row.point);
        }
        if (!active) continue;
        const Slip x = friction_.frictionImpulse(pair, goal, PairFriction::inverted(friction_.frictionResponse(pair, !upperIsB, upperIsB)));
        // Поправка прикладывается только к верхнему телу и в запоминаемую силу трения не входит:
        // иначе тёплый старт следующего отрезка приложил бы её к обоим телам.
        friction_.applyFriction(pair, {x[0] - pair.impulse[0], x[1] - pair.impulse[1], x[2] - pair.impulse[2]}, !upperIsB, upperIsB);
    }
}

} // namespace kp
