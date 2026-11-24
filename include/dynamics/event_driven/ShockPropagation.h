#pragma once
// Завершающий проход по нормалям в связной с основанием опоре. Порядок обхода
// идёт от основания и обратно; поправка учитывает реальные массы обеих сторон.

#include <math/StepMemory.h>
#include <dynamics/event_driven/ContactRows.h>

namespace kp {

class PairFriction;

class ShockPropagation {
public:
    explicit ShockPropagation(ContactRows& state) : state_(state) {}
    ShockPropagation(ContactRows& state, PairFriction&) : ShockPropagation(state) {}

    void pass(const StepArray<Real>& goal);

private:
    void project(size_t i, Real goal);
    ContactRows& state_;
};

} // namespace kp
