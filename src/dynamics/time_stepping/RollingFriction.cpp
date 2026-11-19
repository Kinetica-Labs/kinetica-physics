#include <dynamics/time_stepping/RollingFriction.h>

namespace kp {

void RollingFriction::prepare(const SolverBody& a, const SolverBody& b, Vector3 t1, Vector3 t2) {
    const Vector3 turn1 = a.spinFrom(t1) + b.spinFrom(t1), turn2 = a.spinFrom(t2) + b.spinFrom(t2);
    mass_ = Matrix2x2{t1.dot(turn1), t1.dot(turn2), t1.dot(turn2), t2.dot(turn2)}.inverse();
}

void RollingFriction::warmStart(SolverBody& a, SolverBody& b, Vector3 t1, Vector3 t2) const {
    if (roll_.x != 0 || roll_.y != 0) SolverBody::twist(a, b, roll_.x * t1 + roll_.y * t2);
}

void RollingFriction::solve(SolverBody& a, SolverBody& b, Vector3 t1, Vector3 t2, Real arm, Real load) {
    if (arm == 0) return;
    const Vector3 spin = b.w - a.w;
    Vector2 roll = roll_ - mass_ * Vector2{spin.dot(t1), spin.dot(t2)};
    const Real size = roll.length(), limit = arm * load;
    if (size > limit) roll = (limit / size) * roll;
    SolverBody::twist(a, b, (roll.x - roll_.x) * t1 + (roll.y - roll_.y) * t2);
    roll_ = roll;
}

} // namespace kp
