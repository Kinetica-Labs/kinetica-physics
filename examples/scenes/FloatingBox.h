#pragma once
// Вертикальные колебания куба в спокойной воде. Поверхность воды находится на y = 0.
#include <world/World.h>
#include <algorithm>

namespace floating_box {
using kp::Real;
constexpr Real edge = Real(0.20), waterDensity = 1000, boxDensity = 500;
constexpr Real gravity = Real(9.81), dt = Real(1) / 480;
constexpr Real draft = edge * boxDensity / waterDensity;
constexpr Real equilibriumY = edge / 2 - draft;
constexpr kp::BodyId boxId{0};

inline Real submergedDepth(Real centerY) {
    return std::clamp(edge / 2 - centerY, Real(0), edge);
}

inline Real buoyantForce(Real depth) {
    return waterDensity * gravity * edge * edge * depth; // ρ g V, Н
}

inline kp::World build(Real lift) {
    kp::World world;
    world.gravity = {0, -gravity, 0};
    const kp::Vector3 half{edge / 2, edge / 2, edge / 2}, center{0, equilibriumY + lift, 0};
    const auto box = world.addBody(kp::Body::box(half, boxDensity, center));
    const auto frame = world.addBody(kp::Body{});
    // Направляющая сохраняет вертикальную ориентацию даже при захвате мышью за угол.
    kp::addSlider(world, frame, box, center, {0, 1, 0});
    return world;
}

inline void applyBuoyancy(kp::World& world, double) {
    kp::Body& box = world.body(boxId);
    const Real depth = submergedDepth(box.position.y);
    const kp::Vector3 center = box.position + kp::Vector3{0, (depth - edge) / 2, 0};
    kp::applyForce(box, {0, buoyantForce(depth), 0}, center);
}
} // namespace floating_box
