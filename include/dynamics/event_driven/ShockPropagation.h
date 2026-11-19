#pragma once
// Проход «ударной волны» решателя контактов (Guendelman, Bridson, Fedkiw, «Nonconvex rigid bodies with
// stacking», 2003): пары обходятся снизу вверх, каждая решается заново так, будто нижнее тело
// неподвижно, поправка прикладывается только к верхнему. Импульс системы при этом не сохраняется.

#include <math/StepMemory.h>
#include <dynamics/event_driven/PairFriction.h>

namespace kp {

class ShockPropagation {
public:
    ShockPropagation(ContactRows& state, PairFriction& friction) : state_(state), friction_(friction) {}

    void pass(const StepArray<Real>& goal);

private:
    using Row = ContactRows::Row;
    using Pair = ContactRows::Pair;
    using Slip = ContactRows::Slip;

    ContactRows& state_;
    PairFriction& friction_;
};

} // namespace kp
