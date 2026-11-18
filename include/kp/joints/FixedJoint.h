#pragma once
// Жёсткое сочленение: тела скреплены, ни одной свободы.

#include <kp/joints/Joint.h>

namespace kp {

class FixedJoint final : public Joint {
public:
    FixedJoint(int a, const Body& bodyA, int b, const Body& bodyB, Vector3 anchor) : Joint(a, bodyA, b, bodyB, anchor) {}
    std::unique_ptr<Joint> clone() const override { return std::make_unique<FixedJoint>(*this); }

protected:
    void build(const Motion& motion, std::vector<Bond>& bonds) const override;
};

} // namespace kp
