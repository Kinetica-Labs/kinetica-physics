#pragma once
// Две равные массы вращаются вокруг общего центра в невесомости. Расстояние — 0,5 м.
#include <world/World.h>

namespace rigid_rod {
constexpr kp::Real length = .5, speed = .25;
inline kp::World build() {
    kp::World w;
    w.gravity = {};
    auto left = kp::Body::sphere(.05, 1000, {-length/2,0,0});
    auto right = kp::Body::sphere(.05, 1000, {length/2,0,0});
    left.linearMomentum.y = -speed / left.mass.inverseMass;
    right.linearMomentum.y = speed / right.mass.inverseMass;
    const auto a = w.addBody(left), b = w.addBody(right);
    kp::addDistance(w, a, left.position, b, right.position);
    return w;
}
} // namespace rigid_rod
