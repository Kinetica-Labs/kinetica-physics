#include <kp/joints/SliderJoint.h>

namespace kp {

// Ход: расстояние между точками сочленения двух тел вдоль главной оси тела a.
Real SliderJoint::coordinate(const Body& bodyA, const Body& bodyB) const {
    return (bodyA.orientation * frameA).rotate({1, 0, 0}).dot(bodyB.pose().point(anchorB) - bodyA.pose().point(anchorA));
}

// Сила привода приложена к обоим телам в одной точке — точке сочленения тела b.
void SliderJoint::applyDrive(std::vector<Body>& bodies, Real h) const {
    if (driveForce == 0) return;
    Body &bodyA = bodies[size_t(a)], &bodyB = bodies[size_t(b)];
    const Vector3 push = (h * driveForce) * (bodyA.orientation * frameA).rotate({1, 0, 0});
    const Vector3 at = bodyB.pose().point(anchorB);
    bodyB.applyImpulse(push, at);
    bodyA.applyImpulse(-1 * push, at);
}

void SliderJoint::build(const Motion& m, std::vector<Bond>& bonds) const {
    for (int k = 1; k < 3; ++k) lockTravel(m, k, bonds);
    Bond freedom = travelLine(m, 0);
    freedom.error = 0;
    lockAllTurns(m, bonds);
    addFreedomBonds(m, freedom, bonds);
}

} // namespace kp
