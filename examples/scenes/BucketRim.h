#pragma once
// Два ведра осели. Верхнее опускаем до кромки и отпускаем из покоя.
#include <collision/PairFinder.h>
#include <world/World.h>
#include <stdexcept>

namespace bucket_rim {
using namespace kp;
constexpr BodyId top{3};

inline bool topTouches(const World& world) {
    for (const auto& pair : PairFinder(world.bodies(), world.gravity).pairs(0))
        if (pair.b == int(top) && pair.inContact() && pair.manifold.count) return true;
    return false;
}

inline World releaseOnRim(World base) {
    base.realtime = true;
    for (int frame = 0; frame < 240; ++frame) base.step(Real(1) / 60);
    World world;
    world.realtime = true;
    world.sleeping = false;
    world.shockPropagation = base.shockPropagation;
    world.soft.rollingResistance = 0;
    for (auto body : base.bodies()) {
        body.linearMomentum = body.angularMomentum = {};
        body.linearDamping = body.angularDamping = 0;
        world.addBody(body);
    }
    auto& b = world.body(top);
    b.orientation = Quaternion::fromRotationVector({0, 0, Real(16) * kPi / 180});
    b.position.x += Real(.02);
    b.position.y = world.body(BodyId{2}).position.y + Real(.55);
    const Real start = b.position.y;
    while (!topTouches(world) && start - b.position.y < Real(.56)) b.position.y -= Real(.004);
    if (!topTouches(world)) throw std::runtime_error("missing rim contact");
    Real contact = b.position.y, free = contact + Real(.004);
    for (int i = 0; i < 19; ++i) {
        b.position.y = (contact + free) / 2;
        if (topTouches(world)) contact = b.position.y;
        else free = b.position.y;
    }
    b.position.y = free + Real(1e-5);
    return world;
}
} // namespace bucket_rim
