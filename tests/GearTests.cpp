#include "Check.h"
#include <world/World.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace kp;

namespace {
void gearCoasts(bool realtime, Real speed) {
    World world;
    world.gravity = {};
    world.realtime = realtime;
    const Vector3 shaft{Real(0.6), Real(0.8), 0};
    const auto frame = world.addBody(Body{});
    const auto a = world.addBody(Body::sphere(Real(0.05), 1000));
    const auto b = world.addBody(Body::sphere(Real(0.1), 800, {0, 0, Real(0.15)}));
    // Произвольные начальные метки. Сферы дают известную скалярную инерцию вокруг любой оси.
    world.body(a).orientation = Quaternion::fromRotationVector({Real(0.2), 0, 0});
    world.body(b).orientation = Quaternion::fromRotationVector({0, 0, Real(-0.4)});
    const auto& in = addHinge(world, frame, a, world.body(a).position, shaft);
    const auto& out = addHinge(world, frame, b, world.body(b).position, shaft);
    addGear(world, a, b, 3 * shaft, 17, 31);
    const Real ratio = Real(17) / 31;
    world.body(a).angularMomentum = (speed / world.body(a).mass.inverseInertia.y) * shaft;
    world.body(b).angularMomentum = (-ratio * speed / world.body(b).mass.inverseInertia.y) * shaft;
    const Real energy0 = kineticEnergyOf(world.body(a)) + kineticEnergyOf(world.body(b));
    Real worstAngle = 0, worstEnergy = 0;
    for (int i = 1; i <= 240; ++i) {
        world.step(Real(1) / 120);
        const Real expected = speed * i / 120;
        // Эталон на окружности. В отличие от показания шарнира полный оборот меняет expected на 2π.
        const auto error = [](Real angle) { return std::abs(std::remainder(angle, 2 * kPi)); };
        worstAngle = std::max({worstAngle, error(coordinate(world, in) - expected), error(coordinate(world, out) + ratio * expected)});
        const Real energy = kineticEnergyOf(world.body(a)) + kineticEnergyOf(world.body(b));
        worstEnergy = std::max(worstEnergy, std::abs(energy / energy0 - 1));
    }
    CHECK_NEAR(worstAngle, 0, 1e-6);
    CHECK_NEAR(worstEnergy, 0, 1e-6);
}

void rejectInvalidGear() {
    World world;
    const auto a = world.addBody(Body::sphere(1, 1)), b = world.addBody(Body::sphere(1, 1));
    const auto rejected = [&](BodyId other, Vector3 axis, int na, int nb) {
        try { addGear(world, a, other, axis, na, nb); }
        catch (const std::invalid_argument&) { return true; }
        return false;
    };
    CHECK(rejected(b, {0,1,0}, 0, 40), "zero input teeth");
    CHECK(rejected(b, {0,1,0}, 20, -1), "negative output teeth");
    CHECK(rejected(b, {}, 20, 40), "zero shaft axis");
    CHECK(rejected(a, {0,1,0}, 20, 40), "same rotor");
    CHECK(rejected(b, {0,1,std::numeric_limits<Real>::quiet_NaN()}, 20, 40), "non-finite shaft");
    CHECK(world.joints().empty(), "invalid gear inserted into world");
}

void gearMeasuresPhase() {
    World world;
    const auto a = world.addBody(Body::sphere(1, 1)), b = world.addBody(Body::sphere(1, 1));
    const Vector3 shaft{0, 0, 1};
    const auto& gear = addGear(world, a, b, shaft, 17, 31);
    // Прошли несколько целых оборотов, затем выход отстал на 0,01 рад.
    // Любое из двух представлений поворота (q и −q) должно дать ту же ошибку.
    const Real input = Real(11.2) * kPi, offset = Real(-0.01);
    world.body(a).orientation = Quaternion::fromRotationVector(input * shaft);
    const Quaternion output = Quaternion::fromRotationVector((offset - Real(17) / 31 * input) * shaft);
    for (Real sign : {Real(-1), Real(1)}) {
        world.body(b).orientation = {sign * output.w, sign * output.x, sign * output.y, sign * output.z};
        const auto rows = gear.bonds(world.bodies(), world.bodies());
        CHECK(rows.size() == 1, "one angular constraint");
        if (rows.size() == 1) CHECK_NEAR(rows[0].error, offset, 1e-8);
    }
}
}

void gearTests() {
    for (bool realtime : {false, true})
        for (Real speed : {Real(-17), Real(17)}) gearCoasts(realtime, speed);
    rejectInvalidGear();
    gearMeasuresPhase();
}
