#pragma once

#include <collision/RayCast.h>
#include <dynamics/WheelContact.h>

namespace kp {
class World;
enum class BodyId : int;

struct WheelMount {
    Vector3 anchor; // м в осях шасси: +Y вверх, +Z вперёд
    bool steers;
};

struct VehicleInput {
    Real rimSpeed = 0; // м/с, заданная скорость обода каждого ведущего колеса
    Real steering = 0; // рад в пределах ±π/2, положительный угол направляет колёса вправо
};

struct VehicleWheel {
    Vector3 centre, axle; // положение и ось для отображения в мире
    RayHit ground;
    Vector3 force; // Н, сила дороги на шасси в точке ground.point
};

// Виртуальные колёса с приводом скорости. Один луч измеряет ход каждой подвески.
class VehicleConstraint {
public:
    VehicleConstraint(BodyId chassis, std::vector<WheelMount> mounts, Real radius, Real restLength, WheelSettings tire);
    BodyId chassis() const { return chassis_; }
    size_t wheelCount() const { return mounts_.size(); }
    Real radius() const { return radius_; }
    VehicleWheel wheel(const World& world, size_t index, VehicleInput input) const;
    void apply(World& world, VehicleInput input) const;

private:
    BodyId chassis_;
    std::vector<WheelMount> mounts_;
    Real radius_, restLength_; // м, радиус и длина ненагруженной подвески
    WheelContact tire_;
};
} // namespace kp
