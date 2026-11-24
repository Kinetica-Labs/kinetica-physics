#pragma once
// Четверть машины на вертикальной направляющей: вся подрессоренная масса приходится на одно колесо.
#include <dynamics/WheelContact.h>
#include <world/World.h>
namespace wheel_bench {
constexpr kp::Real mass = 10, stiffness = 2500, gravity = 9.81, restHeight = .20, lift = .01;
constexpr kp::Real equilibrium = restHeight - mass * gravity / stiffness;
constexpr kp::BodyId road{0}, chassis{1};
inline kp::World build() {
    kp::World w;
    const auto base = w.addBody(kp::Body::box({.35,.015,.2},0,{0,-.015,0}));
    constexpr kp::Real hx = .10, hy = .035, hz = .09;
    const auto body = w.addBody(kp::Body::box({hx,hy,hz}, mass / (8*hx*hy*hz), {0,equilibrium+lift,0}));
    kp::addSlider(w, base, body, w.body(body).position, {0,1,0});
    auto tire = kp::Body::cylinder(.035, .045, 0, {0,.045,0});
    tire.orientation = kp::Quaternion::fromRotationVector({kp::kPi/2,0,0});
    w.addBody(tire);
    return w;
}
inline const kp::WheelContact& tire() {
    static const kp::WheelContact model({stiffness, 2 * kp::sqrt(mass * stiffness), 400, 500, .8, .6});
    return model;
}
inline void load(kp::World& w, double) {
    auto& body = w.body(chassis);
    auto& ground = w.body(road);
    const kp::Vector3 point{body.position.x, 0, body.position.z};
    const auto slip = kp::pointVelocity(body, point) - kp::pointVelocity(ground, point);
    const auto f = tire().forces(restHeight - body.position.y, -slip.y, slip.x, slip.z);
    const kp::Vector3 force{f.longitudinal, f.normal, f.lateral};
    kp::applyForce(body, force, point);
    kp::applyForce(ground, -force, point);
}
} // namespace wheel_bench
