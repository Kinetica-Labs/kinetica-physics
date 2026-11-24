#pragma once
// Жёсткая невесомая тяга. Фиксирует расстояние между двумя точками, повороты тел свободны.
#include <joints/Joint.h>

namespace kp {
class DistanceJoint final : public Joint {
public:
    // Точки в мировых координатах. Длина тяги равна их расстоянию при сборке, м.
    DistanceJoint(int a, const Body& bodyA, int b, const Body& bodyB, Vector3 pointA, Vector3 pointB);
    std::unique_ptr<Joint> clone() const override { return std::make_unique<DistanceJoint>(*this); }
    Real length() const { return length_; }

protected:
    void build(const Motion& motion, std::vector<Bond>& bonds) const override;

private:
    Real length_;
    Vector3 initialAxisA_; // направление в теле A, если после перемещения точки совпали
};
} // namespace kp
