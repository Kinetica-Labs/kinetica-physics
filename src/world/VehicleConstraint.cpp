#include <world/VehicleConstraint.h>
#include <world/World.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace kp {
VehicleConstraint::VehicleConstraint(BodyId chassis, std::vector<WheelMount> mounts, Real radius, Real restLength, WheelSettings tire)
    : chassis_(chassis), mounts_(std::move(mounts)), radius_(radius), restLength_(restLength), tire_(tire) {
    if (int(chassis) < 0 || mounts_.empty() || !(radius > 0 && restLength > 0) || !std::isfinite(radius + restLength))
        throw std::invalid_argument("invalid vehicle dimensions or chassis");
    for (const WheelMount& mount : mounts_)
        if (!std::isfinite(mount.anchor.lengthSquare())) throw std::invalid_argument("invalid wheel mount");
}

VehicleWheel VehicleConstraint::wheel(const World& world, size_t index, VehicleInput input) const {
    const WheelMount& mount = mounts_.at(index);
    const Body& body = world.bodies().at(size_t(chassis_));
    if (!std::isfinite(input.rimSpeed) || !(abs(input.steering) <= kPi / 2)) throw std::invalid_argument("invalid drive input");
    const SinCos angle = sinCos(mount.steers ? input.steering : 0);
    const Vector3 up = body.orientation.rotate({0, 1, 0});
    const Vector3 heading = body.orientation.rotate({angle.sin, 0, angle.cos});
    const Vector3 anchor = frameOf(body).point(mount.anchor);
    VehicleWheel result{anchor - restLength_ * up, up.cross(heading), {}, {}};
    const Real reach = restLength_ + radius_;
    const RayHit hit = castRay(world.bodies(), {anchor, -up, reach}, &body);
    if (!hit) return result;
    const Vector3 normal = hit.normal;
    const Real incidence = up.dot(normal);
    if (incidence <= 0) return result;
    result.ground = hit;
    result.centre = anchor - std::max(Real(0), hit.distance - radius_) * up;
    // Продольная ось шины лежит в касательной плоскости дороги.
    const Vector3 tangent = heading - heading.dot(normal) * normal;
    const Real length = tangent.length();
    if (length <= 64 * kEpsilon) return result;
    const Vector3 forward = (1 / length) * tangent;
    result.axle = normal.cross(forward);
    const Body& road = world.body(BodyId(hit.body));
    const Vector3 slip = pointVelocity(body, hit.point) - pointVelocity(road, hit.point);
    const Real compression = (reach - hit.distance) * incidence; // м по нормали дороги
    const auto load = tire_.forces(compression, -slip.dot(normal), slip.dot(forward) - input.rimSpeed, slip.dot(result.axle));
    result.force = load.normal * normal + load.longitudinal * forward + load.lateral * result.axle;
    return result;
}

void VehicleConstraint::apply(World& world, VehicleInput input) const {
    for (size_t i = 0; i < wheelCount(); ++i) {
        const VehicleWheel contact = wheel(world, i, input);
        if (!contact.ground || contact.force.lengthSquare() == 0) continue;
        const BodyId support{contact.ground.body};
        if (world.asleep(chassis_)) world.wake(chassis_);
        if (world.asleep(support)) world.wake(support);
        applyForce(world.body(chassis_), contact.force, contact.ground.point);
        applyForce(world.body(support), -contact.force, contact.ground.point);
    }
}
} // namespace kp
