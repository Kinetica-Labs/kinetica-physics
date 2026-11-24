#include "Check.h"
#include "../examples/scenes/Vehicle.h"
#include <cmath>
#include <cstdio>

namespace {
using namespace kp;
using namespace vehicle_demo;

void loadAndLift() {
    World world = build();
    const Real weight = mass * gravity;
    for (size_t i = 0; i < car().wheelCount(); ++i) {
        const auto contact = car().wheel(world, i, {});
        CHECK(contact.ground.body == 0, "колесо %zu должно попасть в дорогу", i);
        CHECK_NEAR(contact.force.y, weight / 4, weight * .001 / 4);
    }
    car().apply(world, {});
    const Body& body = world.body(chassis);
    CHECK_NEAR((body.force - Vector3{0, weight, 0}).length(), 0, weight * .001);
    CHECK_NEAR(body.torque.length(), 0, 1e-8);
    std::printf("vehicle,normal_load_N=%.12g,weight_N=%.12g\n", body.force.y, weight);
    world.body(chassis).position.y += 1;
    for (size_t i = 0; i < car().wheelCount(); ++i) {
        const auto contact = car().wheel(world, i, {4, .35});
        CHECK(!contact.ground && contact.force.lengthSquare() == 0, "колесо %zu передаёт тягу в воздухе", i);
    }
}

void movingRoad() {
    World world = build();
    Body& road = world.body(BodyId{0});
    road = Body::box({24, .25, 24}, 100, {0, -.25, 0});
    road.linearMomentum = (1 / road.mass.inverseMass) * Vector3{0, 0, 1};
    car().apply(world, {});
    const Body& body = world.body(chassis);
    const Vector3 moment = body.torque + road.torque + body.position.cross(body.force) + road.position.cross(road.force);
    CHECK(body.force.z > 0, "движущаяся дорога должна тянуть шасси вперёд");
    CHECK_NEAR((body.force + road.force).length(), 0, 1e-9);
    CHECK_NEAR(moment.length(), 0, 1e-9);
}

void slopeNormal() {
    World world = build();
    Body& road = world.body(BodyId{0});
    road.orientation = Quaternion::fromRotationVector({0, 0, .12});
    const auto contact = car().wheel(world, 1, {});
    CHECK(contact.ground.body == 0, "луч пропустил наклонную опору");
    CHECK_NEAR((contact.ground.normal - road.orientation.rotate({0, 1, 0})).length(), 0, 1e-6);
}

void advance(World& world, VehicleInput input, int ticks) {
    for (int i = 0; i < ticks; ++i) {
        car().apply(world, input);
        world.step(Real(1) / 480);
    }
}

void steeringAndBrake(bool realtime) {
    World straight = build(), left = build(), right = build();
    for (World* world : {&straight, &left, &right}) {
        world->realtime = realtime;
        world->soft.substeps = world->soft.contactUpdates = 1;
    }
    advance(straight, {4, 0}, 360);
    advance(left, {4, -.35}, 360);
    advance(right, {4, .35}, 360);
    const Vector3 a = straight.body(chassis).position, b = left.body(chassis).position, c = right.body(chassis).position;
    CHECK(a.z > 1 && kp::abs(a.x) < .002, "прямой ход x=%.6g z=%.6g м", a.x, a.z);
    CHECK(b.x < -.1 && c.x > .1 && b.z > .5 && c.z > .5, "повороты x=%.6g/%.6g м", b.x, c.x);
    CHECK_NEAR(b.x + c.x, 0, .002);
    advance(straight, {}, 480);
    CHECK(straight.body(chassis).linearVelocity().length() < .10, "машина не затормозила");
    advance(straight, {-3, 0}, 240);
    CHECK(straight.body(chassis).linearVelocity().z < -1, "нет заднего хода");
    std::printf("vehicle,fast=%d,straight_z_m=%.9g,left_x_m=%.9g,right_x_m=%.9g\n", int(realtime), a.z, b.x, c.x);
}
} // namespace

void vehicleTests() {
    loadAndLift();
    movingRoad();
    slopeNormal();
    steeringAndBrake(false);
    steeringAndBrake(true);
}
