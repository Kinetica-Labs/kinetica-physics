#pragma once
// Цилиндрическое сочленение: поворот вокруг главной оси и перемещение вдоль неё.

#include <kp/joints/Joint.h>

namespace kp {

class CylindricalJoint final : public Joint {
public:
    CylindricalJoint(int a, const Body& bodyA, int b, const Body& bodyB, Vector3 anchor, Vector3 axis) : Joint(a, bodyA, b, bodyB, anchor, axis) {}
    std::unique_ptr<Joint> clone() const override { return std::make_unique<CylindricalJoint>(*this); }

protected:
    void build(const Motion& motion, std::vector<Bond>& bonds) const override;
};

} // namespace kp
