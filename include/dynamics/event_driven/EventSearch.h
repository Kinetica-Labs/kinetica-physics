#pragma once
// Поиск событий, которые обрывают отрезок точного шага: удар, предел хода сочленения, предел
// поворота тела в контакте. Силы на отрезке постоянны (rates), тела движутся по ним.

#include <collision/ContactPair.h>
#include <joints/JointSet.h>
#include <dynamics/event_driven/Rate.h>

#include <vector>

namespace kp {

class EventSearch {
public:
    EventSearch(const std::vector<Body>& bodies, const JointSet& joints) : bodies_(bodies), joints_(joints) {}
    Real earliestImpact(const std::vector<ContactPair>& pairs, const std::vector<Rate>& rates, Real h) const;
    Real earliestLimit(const std::vector<Rate>& rates, Real h) const;
    Real contactTurn(const std::vector<ContactPair>& pairs, const std::vector<Rate>& rates, Real t) const;

private:
    const std::vector<Body>& bodies_;
    const JointSet& joints_;
};

} // namespace kp
