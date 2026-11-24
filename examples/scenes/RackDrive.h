#pragma once
// Шестерня радиусом 20 мм на шарнире и рейка на ползуне. Направления заданы в мире.
#include <world/World.h>

namespace rack_drive {
constexpr kp::Real radius = .02;
constexpr kp::Vector3 shaft{0, 1, 0}, travel{1, 0, 0};
constexpr kp::BodyId pinion{1}, rack{2};
inline kp::World build() {
    kp::World w;
    w.gravity = {};
    const auto base = w.addBody(kp::Body{});
    const auto gear = w.addBody(kp::Body::cylinder(.008, radius, 1000));
    const auto bar = w.addBody(kp::Body::box({.085,.008,.006}, 1000, {0,0,radius+.006}));
    kp::addHinge(w, base, gear, {}, shaft).holdSpeed = true;
    kp::addSlider(w, base, bar, w.body(bar).position, travel);
    kp::addRackAndPinion(w, bar, gear, {}, shaft, travel, radius);
    return w;
}
// Пол-оборота в секунду. Каждые полсекунды меняется знак скорости привода.
inline void drive(kp::World& w, double time) {
    const auto halfSeconds = static_cast<long long>(2 * time + 1e-10);
    w.joint<kp::HingeJoint>(0).driveSpeed = halfSeconds % 2 ? -kp::kPi : kp::kPi;
}
} // namespace rack_drive
