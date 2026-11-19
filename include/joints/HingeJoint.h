#pragma once
// Шарнир: свободен только поворот вокруг главной оси. Привод, трение, пружина и пределы — SingleFreedomJoint.

#include <joints/SingleFreedomJoint.h>

namespace kp {

class HingeJoint final : public SingleFreedomJoint {
public:
    HingeJoint(int a, const Body& bodyA, int b, const Body& bodyB, Vector3 anchor, Vector3 axis) : SingleFreedomJoint(a, bodyA, b, bodyB, anchor, axis) {}
    std::unique_ptr<Joint> clone() const override { return std::make_unique<HingeJoint>(*this); }

    using SingleFreedomJoint::coordinate;
    Real coordinate(const Body& bodyA, const Body& bodyB) const override;
    bool angular() const override { return true; }
    void applyDrive(std::vector<Body>& bodies, Real h) const override;

protected:
    void build(const Motion& motion, std::vector<Bond>& bonds) const override;
};

} // namespace kp
