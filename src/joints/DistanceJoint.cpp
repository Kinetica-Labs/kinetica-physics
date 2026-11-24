#include <joints/DistanceJoint.h>
#include <cmath>
#include <stdexcept>

namespace kp {
DistanceJoint::DistanceJoint(int first, const Body& bodyA, int second, const Body& bodyB, Vector3 pointA, Vector3 pointB)
    : length_((pointB - pointA).length()) {
    if (first == second || !std::isfinite(length_) || length_ <= 0)
        throw std::invalid_argument("distance joint requires two bodies and distinct finite anchors");
    a = first;
    b = second;
    anchorA = bodyA.pose().localPoint(pointA);
    anchorB = bodyB.pose().localPoint(pointB);
    initialAxisA_ = bodyA.orientation.rotateInverse((1 / length_) * (pointB - pointA));
}

void DistanceJoint::build(const Motion& m, std::vector<Bond>& bonds) const {
    const Vector3 span = m.pb - m.pa;
    const Real distance = span.length();
    const bool directed = distance > kEpsilon * length_;
    const Vector3 n = directed ? (1 / distance) * span : m.bodyA->orientation.rotate(initialAxisA_);
    const Vector3 armA = m.pa - m.bodyA->position, armB = m.armB;
    const Vector3 relative = m.pointB - (m.pointA - m.wa.cross(span));
    const Vector3 tangent = relative - n.dot(relative) * n;
    const Vector3 accelerationA = m.spinUpA.cross(armA) + m.wa.cross(m.wa.cross(armA));
    const Vector3 accelerationB = m.spinUpB.cross(armB) + m.wb.cross(m.wb.cross(armB));
    // d²|span|/dt² = n·(aB−aA) + |v поперёк тяги|²/|span|, м/с².
    const Real drift = n.dot(accelerationB - accelerationA) + (directed ? tangent.lengthSquare() / distance : Real(0));
    bonds.push_back({a, b, n, armA.cross(n), armB.cross(n), distance - length_, drift});
}
} // namespace kp
