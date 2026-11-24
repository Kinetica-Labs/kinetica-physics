#include <dynamics/time_stepping/CentreFriction.h>

#include <algorithm>

namespace kp {

void CentreFriction::prepare(const SolverBody& a, const SolverBody& b, const Frame& at, const Carried& carried) {
    rub_ = {carried.rubbed.dot(at.tangent1), carried.rubbed.dot(at.tangent2)}, twistImpulse_ = carried.twist;
    const Vector3 a1 = at.centreA.cross(at.tangent1), a2 = at.centreA.cross(at.tangent2), b1 = at.centreB.cross(at.tangent1), b2 = at.centreB.cross(at.tangent2);
    const Real k11 = a.inverseMass + b.inverseMass + a1.dot(a.spinFrom(a1)) + b1.dot(b.spinFrom(b1));
    const Real k22 = a.inverseMass + b.inverseMass + a2.dot(a.spinFrom(a2)) + b2.dot(b.spinFrom(b2));
    const Real k12 = a1.dot(a.spinFrom(a2)) + b1.dot(b.spinFrom(b2));
    const Real around = at.normal.dot(a.spinFrom(at.normal) + b.spinFrom(at.normal));
    tangentMass_ = Matrix2x2{k11, k12, k12, k22}.inverse(), twistMass_ = around > 0 ? 1 / around : 0;
    rolling_.prepare(a, b, at.tangent1, at.tangent2);
}

void CentreFriction::warmStart(SolverBody& a, SolverBody& b, const Frame& at) const {
    SolverBody::twist(a, b, twistImpulse_ * at.normal);
    SolverBody::push(a, b, at.centreA, at.centreB, rub_.x * at.tangent1 + rub_.y * at.tangent2);
    rolling_.warmStart(a, b, at.tangent1, at.tangent2);
}

void CentreFriction::solveTwist(SolverBody& a, SolverBody& b, const Frame& at, Real leverLoad) {
    const Real turning = std::max(-coefficient * leverLoad, std::min(coefficient * leverLoad, twistImpulse_ - twistMass_ * at.normal.dot(b.w - a.w)));
    SolverBody::twist(a, b, (turning - twistImpulse_) * at.normal);
    twistImpulse_ = turning;
}

void CentreFriction::setTangent(SolverBody& a, SolverBody& b, const Frame& at, Vector2 rub, Real load) {
    const Real size = rub.length(), limit = coefficient * load;
    if (size > limit) rub = (limit / size) * rub;
    SolverBody::push(a, b, at.centreA, at.centreB, (rub.x - rub_.x) * at.tangent1 + (rub.y - rub_.y) * at.tangent2);
    rub_ = rub;
}

void CentreFriction::solve(SolverBody& a, SolverBody& b, const Frame& at, Real load, Real leverLoad, Real rollingArm) {
    solveTwist(a, b, at, leverLoad);
    const Vector3 slip = b.v + b.w.cross(at.centreB) - a.v - a.w.cross(at.centreA);
    const Real s1 = slip.dot(at.tangent1), s2 = slip.dot(at.tangent2);
    setTangent(a, b, at, rub_ - tangentMass_ * Vector2{s1, s2}, load);
    rolling_.solve(a, b, at.tangent1, at.tangent2, rollingArm, load);
}

} // namespace kp
