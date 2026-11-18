#include <kp/joints/HingeJoint.h>

namespace kp {

// Угол между осями y двух тел вокруг главной оси тела a.
Real HingeJoint::coordinate(const Body& bodyA, const Body& bodyB) const {
    const Quaternion inA = bodyA.orientation * frameA, inB = bodyB.orientation * frameB;
    const Vector3 yA = inA.rotate({0, 1, 0}), zA = inA.rotate({0, 0, 1}), yB = inB.rotate({0, 1, 0});
    return atan2(yB.dot(zA), yB.dot(yA));
}

// Момент привода: телу b — вдоль главной оси, телу a — противоположный.
void HingeJoint::applyDrive(std::vector<Body>& bodies, Real h) const {
    if (driveForce == 0) return;
    Body &bodyA = bodies[size_t(a)], &bodyB = bodies[size_t(b)];
    const Vector3 push = (h * driveForce) * (bodyA.orientation * frameA).rotate({1, 0, 0});
    if (!bodyB.isStatic()) bodyB.angularMomentum += push;
    if (!bodyA.isStatic()) bodyA.angularMomentum += -1 * push;
}

void HingeJoint::build(const Motion& m, std::vector<Bond>& bonds) const {
    for (int k = 0; k < 3; ++k) lockTravel(m, k, bonds);
    lockTurnsAcrossAxis(m, bonds);
    addFreedomBonds(m, turnLine(m, 0), bonds);
}

} // namespace kp
