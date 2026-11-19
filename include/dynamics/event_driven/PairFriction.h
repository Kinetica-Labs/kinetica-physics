#pragma once
// Трение пары тел в решателе контактов: сила в касательной плоскости и момент вокруг нормали в центре
// точек касания пары (ContactRows::Pair). Закон Кулона — конус: |λt| <= μ λn.

#include <math/StepMemory.h>
#include <dynamics/event_driven/ContactRows.h>

namespace kp {

class PairFriction {
public:
    using Row = ContactRows::Row;
    using Pair = ContactRows::Pair;
    using Slip = ContactRows::Slip;
    using Response = ContactRows::Response;
    using Line = ContactRows::Line;
    using Entry = ContactSystem::Entry;
    using System = ContactSystem;

    explicit PairFriction(ContactRows& state) : state_(state) {}

    // Трение пары: центр точек касания, матрица отклика и скольжение до действия сил.
    Pair frictionOf(const ContactPair& contact, size_t first, size_t last) const;
    // Матрица отклика трения пары; withA и withB — какие из двух тел подвижны.
    Response frictionResponse(const Pair& pair, bool withA, bool withB) const;
    // Обратная к симметричной положительной матрице 3 × 3.
    static Response inverted(const Response& m);
    Slip slip(const Pair& pair) const;
    void applyFriction(const Pair& pair, const Slip& impulse, bool toA, bool toB);
    Slip frictionImpulse(const Pair& pair, const StepArray<Real>& goal, const Response& inverse) const;
    Real solveFriction(Pair& pair, const StepArray<Real>& goal);

private:
    ContactRows& state_;
};

} // namespace kp
