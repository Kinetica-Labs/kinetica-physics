#pragma once
// Поиск событий, которые обрывают отрезок точного шага: удар, предел хода сочленения, предел
// поворота тела в контакте. Силы на отрезке постоянны (rates), тела движутся по ним.

#include <math/StepMemory.h>
#include <collision/ContactPair.h>
#include <joints/JointSet.h>
#include <dynamics/event_driven/Rate.h>

#include <vector>

namespace kp {

class EventSearch {
public:
    EventSearch(const std::vector<Body>& bodies, const JointSet& joints) : bodies_(bodies), joints_(joints) {}
    Real earliestImpact(const std::vector<ContactPair>& pairs, const StepArray<Rate>& rates, Real h) const;
    Real earliestLimit(const StepArray<Rate>& rates, Real h) const;
    Real contactTurn(const std::vector<ContactPair>& pairs, const StepArray<Rate>& rates, Real t) const;

private:
    Real impactOf(const ContactPair& pair, const Rate& ra, const Rate& rb, Real h, Real earliest) const;
    const std::vector<Body>& bodies_;
    const JointSet& joints_;
};

} // namespace kp
