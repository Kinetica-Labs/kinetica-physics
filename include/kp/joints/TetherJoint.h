#pragma once
// Упругая привязь: точки двух тел стянуты пружиной с демпфером, повороты свободны. Так тело тянут
// мышью: жёсткая связь с рукой рвала бы стопки и проталкивала тело сквозь стены, пружина тянет с
// конечной силой. Отпущенная привязь (жёсткость и затухание нулевые) ничего не держит.

#include <kp/joints/Joint.h>

namespace kp {

class TetherJoint final : public Joint {
public:
    Real stiffness = 0; // Н/м
    Real damping = 0;   // Н·с/м

    // pointA и pointB — точки тел в мире, которые привязь стягивает.
    TetherJoint(int a, const Body& bodyA, Vector3 pointA, int b, const Body& bodyB, Vector3 pointB, Real stiffness, Real damping);
    std::unique_ptr<Joint> clone() const override { return std::make_unique<TetherJoint>(*this); }
    // Привязь стягивает тела, которые друг друга не касаются: столкновений она не отменяет.
    bool collides() const override { return true; }
    bool pushes() const override { return stiffness > 0; }

protected:
    void build(const Motion& motion, std::vector<Bond>& bonds) const override;
};

} // namespace kp
