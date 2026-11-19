#pragma once
// Тёплый старт решателя контактов: решение начинается с сил прошлого отрезка. Если прошлые силы на
// новой геометрии добавили бы кинетическую энергию, они уменьшаются в α раз, где α — наилучший масштаб
// вдоль них.

#include <math/StepMemory.h>
#include <dynamics/event_driven/PairFriction.h>

namespace kp {

class WarmStart {
public:
    WarmStart(ContactRows& state, PairFriction& friction) : state_(state), friction_(friction) {}

    void apply(const StepArray<Real>& goal, const ContactForces& previous, Real h);

private:
    using Row = ContactRows::Row;
    using Pair = ContactRows::Pair;
    using Slip = ContactRows::Slip;

    ContactRows& state_;
    PairFriction& friction_;
};

} // namespace kp
