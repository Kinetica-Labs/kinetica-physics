#pragma once
// Идеальная реечная передача. Ползун задаёт ход рейки, шарнир — ось шестерни.
// На общей раме s = r θ: положительный поворот шестерни ведёт рейку по направлению хода.
#include <joints/Joint.h>

namespace kp {
class RackAndPinionJoint final : public Joint {
public:
    // rack — тело A, pinion — B. Точка на оси и два направления заданы в мире, делительный радиус — в метрах.
    RackAndPinionJoint(int rack, const Body& bodyA, int pinion, const Body& bodyB, Vector3 pivot,
                       Vector3 shaft, Vector3 travel, Real pitchRadius);
    std::unique_ptr<Joint> clone() const override { return std::make_unique<RackAndPinionJoint>(*this); }

protected:
    void build(const Motion& motion, std::vector<Bond>& bonds) const override;

private:
    Real radius_;
};
} // namespace kp
