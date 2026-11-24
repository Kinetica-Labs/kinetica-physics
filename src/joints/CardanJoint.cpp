#include <joints/CardanJoint.h>

namespace kp {

CardanJoint::CardanJoint(int a_, const Body& bodyA, int b_, const Body& bodyB, Vector3 anchor, Vector3 axis, Vector3 crossAxis) {
    a = a_, b = b_;
    anchorA = bodyA.pose().localPoint(anchor), anchorB = bodyB.pose().localPoint(anchor);
    // Оси сочленения в мире: x — вал, y — крестовина (её составляющая вдоль вала отброшена).
    const Vector3 x = (1 / axis.length()) * axis;
    const Vector3 across = crossAxis - crossAxis.dot(x) * x;
    const Vector3 y = (1 / across.length()) * across;
    // Сначала совмещаем оси x, затем поворотом вокруг x совмещаем оси y.
    const Quaternion first = turningXTo(x);
    const Vector3 yFirst = first.rotate({0, 1, 0});
    const Real c = yFirst.dot(y), s = yFirst.cross(y).dot(x); // косинус и синус поворота вокруг x
    const Real half = sqrt((1 + c) / 2);
    const Quaternion frame = half > Real(1e-6) ? (Quaternion{half, s / (2 * half) * x.x, s / (2 * half) * x.y, s / (2 * half) * x.z} * first).unit() : (Quaternion{0, x.x, x.y, x.z} * first).unit();
    frameA = bodyA.orientation.conjugate() * frame, frameB = bodyB.orientation.conjugate() * frame;
}

// Крестовина: ось z тела b обязана оставаться перпендикулярной оси y тела a. Та же связь, что у
// шарнира, для другой пары осей: C = z_b · y_a, dC/dt = (ω_b − ω_a) · (z_b × y_a).
void CardanJoint::build(const Motion& m, std::vector<Bond>& bonds) const {
    for (int k = 0; k < 3; ++k) lockTravel(m, k, bonds);
    const Vector3 axisB = m.frameB.rotate({0, 0, 1}), u = m.frameA.rotate({0, 1, 0}), n = axisB.cross(u);
    const Vector3 turning = m.wb.cross(axisB).cross(u) + axisB.cross(m.wa.cross(u)); // dn/dt
    bonds.push_back({a, b, {}, n, n, axisB.dot(u), (m.spinUpB - m.spinUpA).dot(n) + (m.wb - m.wa).dot(turning)});
}

} // namespace kp
