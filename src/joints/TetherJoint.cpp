#include <joints/TetherJoint.h>

namespace kp {

TetherJoint::TetherJoint(int a_, const Body& bodyA, int b_, const Body& bodyB, Vector3 pointA, Vector3 pointB, Real stiffness_, Real damping_)
    : Joint(a_, bodyA, b_, bodyB, pointB), stiffness(stiffness_), damping(damping_) {
    anchorA = bodyA.pose().localPoint(pointA);
}

void TetherJoint::build(const Motion& m, std::vector<Bond>& bonds) const {
    if (!(stiffness > 0 || damping > 0)) return;
    for (int k = 0; k < 3; ++k) {
        Bond spring = travelLine(m, k);
        spring.soft = true, spring.stiffness = stiffness, spring.damping = damping;
        spring.stretch = spring.error, spring.error = 0;
        bonds.push_back(spring);
    }
}

} // namespace kp
