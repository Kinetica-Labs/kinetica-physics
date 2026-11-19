#pragma once
// Карданное сочленение: вал axis и крестовина cross; кручение вала передаётся, два поворота свободны.

#include <joints/Joint.h>

namespace kp {

class CardanJoint final : public Joint {
public:
    CardanJoint(int a, const Body& bodyA, int b, const Body& bodyB, Vector3 anchor, Vector3 axis, Vector3 cross);
    std::unique_ptr<Joint> clone() const override { return std::make_unique<CardanJoint>(*this); }

protected:
    void build(const Motion& motion, std::vector<Bond>& bonds) const override;
};

} // namespace kp
