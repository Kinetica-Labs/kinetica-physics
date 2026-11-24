#pragma once

#include "../examples/scenes/GripGeometry.h"
#include <world/World.h>

namespace threshold_grip {

constexpr grip::Load load{0.5, 10, 10};
constexpr kp::BodyId left{1}, right{2}, cube{3};

inline kp::World build() {
    kp::World world;
    world.gravity = {};
    world.soft.coupledFriction = true;
    world.soft.reuseEquilibrium = true;
    const auto base = world.addBody(kp::Body{});
    for (const auto& box : {grip::leftJaw, grip::rightJaw, grip::cube}) {
        const kp::Vector3 half{kp::Real(box.halfX), kp::Real(box.halfY), kp::Real(box.halfZ)};
        auto body = kp::Body::box(half, kp::Real(box.density), {kp::Real(box.x), 0, 0});
        body.friction = kp::Real(load.friction);
        world.addBody(body);
    }
    for (auto jaw : {left, right})
        kp::addSlider(world, base, jaw, world.body(jaw).position, {1, 0, 0});
    return world;
}

inline void applyLoad(kp::World& world, double time) {
    world.body(left).force.x += kp::Real(load.squeeze);
    world.body(right).force.x -= kp::Real(load.squeeze);
    if (time >= 0.25) world.body(cube).force.y -= kp::Real(load.down);
}

inline double dropMm(const kp::World& world) {
    const auto jawY = (world.body(left).position.y + world.body(right).position.y) / 2;
    return 1000.0 * (jawY - world.body(cube).position.y);
}

} // namespace threshold_grip
