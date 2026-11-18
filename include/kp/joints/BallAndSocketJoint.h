#pragma once
// Шаровое сочленение: точки сочленения двух тел совпадают, повороты свободны.

#include <kp/joints/Joint.h>

namespace kp {

class BallAndSocketJoint final : public Joint {
public:
    BallAndSocketJoint(int a, const Body& bodyA, int b, const Body& bodyB, Vector3 anchor) : Joint(a, bodyA, b, bodyB, anchor) {}
    std::unique_ptr<Joint> clone() const override { return std::make_unique<BallAndSocketJoint>(*this); }

protected:
    void build(const Motion& motion, std::vector<Bond>& bonds) const override;
};

} // namespace kp
