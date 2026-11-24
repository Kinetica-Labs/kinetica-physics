#include <joints/GearJoint.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <stdexcept>

namespace kp {

GearJoint::GearJoint(int first, const Body& bodyA, int second, const Body& bodyB, Vector3 shaft, int teethA, int teethB) {
    if (first == second || teethA <= 0 || teethB <= 0)
        throw std::invalid_argument("gear: distinct wheels and positive tooth counts required");
    const Real scale = std::max({abs(shaft.x), abs(shaft.y), abs(shaft.z)});
    if (scale == 0 || !std::isfinite(shaft.x) || !std::isfinite(shaft.y) || !std::isfinite(shaft.z))
        throw std::invalid_argument("gear: finite nonzero shaft required");
    const Vector3 scaled{shaft.x / scale, shaft.y / scale, shaft.z / scale};
    shaft_ = (1 / scaled.length()) * scaled;
    const int divisor = std::gcd(teethA, teethB);
    turnsA_ = teethA / divisor;
    turnsB_ = teethB / divisor;
    a = first;
    b = second;
    // Joint::bonds получит q(t) q(0)* для каждого колеса в мировых осях.
    frameA = bodyA.orientation.conjugate();
    frameB = bodyB.orientation.conjugate();
}

Real GearJoint::angle(Quaternion rotation) const {
    return 2 * atan2(shaft_.dot({rotation.x, rotation.y, rotation.z}), rotation.w);
}

void GearJoint::build(const Motion& m, std::vector<Bond>& bonds) const {
    // При целом числе оборотов одного колеса его ориентация прежняя. Целые коэффициенты
    // сохраняют фазу по модулю 2π даже при смене знака кватерниона и нескольких оборотах.
    const Real phase = turnsA_ * angle(m.frameA) + turnsB_ * angle(m.frameB);
    const Real turns = phase / (2 * kPi);
    const auto nearest = static_cast<std::int64_t>(turns + (turns < 0 ? Real(-0.5) : Real(0.5)));
    const Real scale = Real(1) / std::max(turnsA_, turnsB_);
    const Real error = scale * (phase - Real(nearest) * (2 * kPi));
    const Vector3 ja = (scale * turnsA_) * shaft_, jb = (scale * turnsB_) * shaft_;
    // Jω = ja·ωA + jb·ωB. Bond сам меняет знак у первого тела. Мощность реакции равна λ Jω.
    bonds.push_back({a, b, {}, -ja, jb, error, ja.dot(m.spinUpA) + jb.dot(m.spinUpB)});
}

} // namespace kp
