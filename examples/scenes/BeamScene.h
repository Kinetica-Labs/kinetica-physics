#pragma once
#include <world/World.h>

namespace support_scene {
inline kp::World beam(kp::Real left, kp::Real right) {
    kp::World world;
    world.soft.coupledFriction = world.soft.reuseEquilibrium = true;
    world.addBody(kp::Body::box({.0125,.05,.16},0));
    world.addBody(kp::Body::box({.65,.025,.15},100,{0,.075,0}));
    world.addBody(kp::Body::box({.1,.1,.1},125*left,{-.45,.20,0}));
    world.addBody(kp::Body::box({.1,.1,.1},125*right,{.45,.20,0}));
    return world;
}
}
