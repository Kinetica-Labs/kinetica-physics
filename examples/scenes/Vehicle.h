#pragma once

#include <world/VehicleConstraint.h>
#include <world/World.h>

namespace vehicle_demo {
constexpr kp::BodyId chassis{1};
constexpr kp::Real mass = 80, gravity = 9.81, radius = .19, travel = .20, stiffness = 9000;
constexpr kp::Vector3 halfSize{.40, .11, .66};

inline const kp::VehicleConstraint& car() {
    static const kp::VehicleConstraint model(chassis,
        {{{-.43, 0, .48}, true}, {{.43, 0, .48}, true}, {{-.43, 0, -.48}, false}, {{.43, 0, -.48}, false}},
        radius, travel, {stiffness, 650, 220, 500, .85, .8});
    return model;
}

inline kp::World build() {
    kp::World world;
    world.addBody(kp::Body::box({24, .25, 24}, 0, {0, -.25, 0}));
    const kp::Real height = radius + travel - mass * gravity / (4 * stiffness);
    const kp::Real volume = 8 * halfSize.x * halfSize.y * halfSize.z;
    world.addBody(kp::Body::box(halfSize, mass / volume, {0, height, 0}));
    for (int z : {3, 6, 9}) {
        world.addBody(kp::Body::box({.12, .32, .12}, 0, {-1.3, .32, kp::Real(z)}));
        world.addBody(kp::Body::box({.12, .32, .12}, 0, {1.3, .32, kp::Real(z)}));
    }
    return world;
}
} // namespace vehicle_demo
