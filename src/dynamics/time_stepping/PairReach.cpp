#include <dynamics/time_stepping/PairReach.h>

#include <collision/PairFinder.h>

#include <algorithm>

namespace kp {

// c простого тела — maxPointSpeed при единичном моменте импульса и нулевой скорости; у части
// составного тела граница та же, что в examinePair: наибольший I⁻¹ на охватывающий радиус части.
PairReach::PairReach(const std::vector<Body>& bodies, const std::vector<ContactPair>& pairs, Vector3 gravity, Real time, Real slack)
    : gravity_(gravity), slack_(slack), least_(bodies.size(), 0), most_(bodies.size(), 0), next_(bodies.size()) {
    for (size_t i = 0; i < bodies.size(); ++i) {
        const Body& body = bodies[i];
        if (body.isStatic() || (!body.shape && !body.parts)) continue;
        if (!body.parts) {
            Body probe = body;
            probe.linearMomentum = {}, probe.angularMomentum = {1, 0, 0};
            least_[i] = most_[i] = maxPointSpeed(probe);
            continue;
        }
        const Vector3 inverse = body.mass.inverseInertia;
        const Real spin = std::max({inverse.x, inverse.y, inverse.z});
        for (size_t k = 0; k < body.parts->size(); ++k) {
            const Real c = spin * ((*body.parts)[k].placement.position.length() + (*body.parts)[k].shape->boundingRadius());
            least_[i] = k ? std::min(least_[i], c) : c, most_[i] = std::max(most_[i], c);
        }
    }
    listed_.reserve(pairs.size());
    for (const ContactPair& pair : pairs) listed_.push_back({pair.a, pair.b, pair.partA, pair.partB});
    std::sort(listed_.begin(), listed_.end());
    allow(bodies, time);
}

void PairReach::allow(const std::vector<Body>& bodies, Real time) {
    allowed_.assign(bodies.size(), {}), spent_.assign(bodies.size(), {});
    for (size_t i = 0; i < bodies.size(); ++i)
        if (!bodies[i].isStatic()) allowed_[i] = {(bodies[i].linearVelocity().length() + gravity_.length() * time) * time, bodies[i].angularMomentum.length() * time};
}

// Путь линеен по c, поэтому выход за границу у какой-нибудь части тела виден на концах отрезка c.
bool PairReach::exceeded(const std::vector<Body>& bodies, Real h) {
    bool any = false;
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i].isStatic()) continue;
        next_[i] = {bodies[i].linearVelocity().length() * h, bodies[i].angularMomentum.length() * h};
        const Real linear = spent_[i].linear + next_[i].linear - allowed_[i].linear, angular = spent_[i].angular + next_[i].angular - allowed_[i].angular;
        any = any || std::max(linear + least_[i] * angular, linear + most_[i] * angular) > slack_;
    }
    return any;
}

void PairReach::spend() {
    for (size_t i = 0; i < spent_.size(); ++i) spent_[i].linear += next_[i].linear, spent_[i].angular += next_[i].angular;
}

StepArray<ContactPair> PairReach::discover(const std::vector<Body>& bodies, const JointSet& joints, Real remaining, Real boundary) {
    PairFinder finder(bodies, gravity_);
    finder.boundary = boundary, finder.known = &listed_; // пары списка заново не проверяются: нужны только новые
    StepArray<ContactPair> fresh;
    for (const ContactPair& pair : finder.pairs(remaining, joints)) fresh.push_back(pair);
    for (const ContactPair& pair : fresh) listed_.push_back({pair.a, pair.b, pair.partA, pair.partB});
    std::sort(listed_.begin(), listed_.end());
    allow(bodies, remaining);
    return fresh;
}

} // namespace kp
