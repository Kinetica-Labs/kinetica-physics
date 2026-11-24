#pragma once
// Две шестерни на параллельных валах. Числа зубьев задают связь углов и скоростей.
#include <world/World.h>

namespace gear_train {
using kp::Real;
constexpr int teethA = 20, teethB = 40;
constexpr Real ratio = Real(teethA) / teethB;
constexpr Real radiusA = Real(0.05), radiusB = Real(0.10), halfHeight = Real(0.01), density = 1000;
constexpr kp::Vector3 shaft{0, 1, 0};
constexpr kp::BodyId inputDisk{1}, outputDisk{2};

inline kp::World build() {
    kp::World world;
    world.gravity = {};
    const auto frame = world.addBody(kp::Body{});
    const auto a = world.addBody(kp::Body::cylinder(halfHeight, radiusA, density));
    const auto b = world.addBody(kp::Body::cylinder(halfHeight, radiusB, density, {radiusA + radiusB, 0, 0}));
    kp::addHinge(world, frame, a, world.body(a).position, shaft);
    kp::addHinge(world, frame, b, world.body(b).position, shaft);
    kp::addGear(world, a, b, shaft, teethA, teethB);
    return world;
}

inline Real diskInertia(Real radius) {
    const Real mass = density * 2 * halfHeight * kp::kPi * radius * radius;
    return mass * radius * radius / 2;
}

inline void drive(kp::World& world, double time) {
    // I = IA + r² IB. Разгон за первую секунду с αA = π рад/с², затем равномерный ход.
    const Real inertia = diskInertia(radiusA) + ratio * ratio * diskInertia(radiusB);
    const Real load = Real(0.001);
    const Real input = (time < 1 ? inertia * kp::kPi : Real(0)) + ratio * load;
    kp::applyTorque(world.body(inputDisk), input * shaft);
    kp::applyTorque(world.body(outputDisk), load * shaft);
}
} // namespace gear_train
