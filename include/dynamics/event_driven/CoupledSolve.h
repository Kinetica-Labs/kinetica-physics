#pragma once
// Решение собранной системы решателя контактов разом: нормали, трение покоящихся пар и связи — одной
// линейной задачей дополнительности перебором активного набора (ActiveSet.h), со срывом пар по
// Кулону и упором приводов в предел силы.

#include <dynamics/event_driven/PairFriction.h>

namespace kp {

class CoupledSolve {
public:
    CoupledSolve(ContactRows& state, PairFriction& friction) : state_(state), friction_(friction) {}

    Real solveTogether(const ContactSystem& system, const std::vector<Real>& goal);
    Real residual(const ContactSystem& system, const std::vector<Real>& goal) const;

private:
    using Row = ContactRows::Row;
    using Pair = ContactRows::Pair;
    using Slip = ContactRows::Slip;
    using Response = ContactRows::Response;
    using Line = ContactRows::Line;
    using Entry = ContactSystem::Entry;
    using System = ContactSystem;

    // Части solveTogether: начальные импульсы и цели, неизвестные, свободные члены, срыв пар
    // по Кулону, упор приводов, приложение ответа.
    void startTogether(const System& system, const std::vector<Real>& goal, std::vector<Real>& current, std::vector<Real>& target) const;
    std::vector<char> unknownsOf(const System& system) const;
    std::vector<Real> freeTerms(const System& system, const std::vector<char>& unknown, const std::vector<Real>& current, const std::vector<Real>& target) const;
    bool releaseSlipping(const System& system, const std::vector<Real>& force);
    bool releaseDrives(const System& system, const std::vector<char>& unknown, std::vector<Real>& current, const std::vector<Real>& target, const std::vector<Real>& force);
    Real applyTogether(const System& system, const std::vector<Real>& force);

    ContactRows& state_;
    PairFriction& friction_;
};

} // namespace kp
