#include <joints/RackAndPinionJoint.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace kp {
namespace {
Vector3 unitAxis(Vector3 v) {
    const Real scale = std::max({abs(v.x), abs(v.y), abs(v.z)});
    if (!(scale > 0) || !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z))
        throw std::invalid_argument("rack and pinion requires finite nonzero axes");
    const Vector3 scaled{v.x / scale, v.y / scale, v.z / scale};
    return (1 / scaled.length()) * scaled;
}
}

RackAndPinionJoint::RackAndPinionJoint(int rack, const Body& bodyA, int pinion, const Body& bodyB,
                                       Vector3 pivot, Vector3 shaft, Vector3 travel, Real pitchRadius)
    : radius_(pitchRadius) {
    const Vector3 axis = unitAxis(shaft);
    const Vector3 direction = unitAxis(travel);
    if (rack == pinion || !(radius_ > 0) || !std::isfinite(2 * kPi * radius_) ||
        !std::isfinite(pivot.x) || !std::isfinite(pivot.y) || !std::isfinite(pivot.z) ||
        abs(direction.dot(axis)) > sqrt(kEpsilon))
        throw std::invalid_argument("rack and pinion requires distinct bodies, finite pivot, positive radius and perpendicular axes");
    a = rack;
    b = pinion;
    anchorA = bodyA.pose().localPoint(pivot);
    anchorB = bodyB.pose().localPoint(pivot);
    const Vector3 side = unitAxis(axis.cross(direction));
    const Quaternion frame = Quaternion::fromAxes(axis, side.cross(axis), side);
    frameA = bodyA.orientation.conjugate() * frame;
    frameB = bodyB.orientation.conjugate() * frame;
}

void RackAndPinionJoint::build(const Motion& m, std::vector<Bond>& bonds) const {
    Bond row = travelLine(m, 1); // перемещение шестерни относительно рейки, −s
    const Bond turn = turnLine(m, 0);
    const Quaternion relative = m.frameA.conjugate() * m.frameB;
    const Real theta = 2 * atan2(relative.x, relative.w);
    const Real phase = row.error + radius_ * theta, circumference = 2 * kPi * radius_;
    // Ориентация задаёт угол по модулю оборота. Ход рейки выбирает ближайший полный оборот.
    row.error = std::remainder(phase, circumference);
    row.angularA += radius_ * turn.angularA;
    row.angularB += radius_ * turn.angularB;
    row.drift += radius_ * turn.drift;
    bonds.push_back(row);
}
} // namespace kp
