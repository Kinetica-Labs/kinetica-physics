// Первый подшаг скользящего сплошного шара на плоскости. Может ли трение обратить скольжение?
#include "StudyFriction.h"
#include <world/World.h>
#include <math/WorkerPool.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {
using namespace kp;
constexpr Real radius = Real(0.1), gravity = 10, h = Real(1) / 480;
constexpr Real speedTolerance = Real(1e-6); // м/с

bool measure(Real speed, Real mu) {
    World world;
    world.soft.coupledFriction = study::coupledFrictionRequested();
    world.realtime = true;
    world.gravity = {0, -gravity, 0};
    world.soft.substeps = world.soft.contactUpdates = 1;
    Body floor = Body::box({1, Real(0.1), 1}, 0, {0, Real(-0.1), 0});
    Body ball = Body::sphere(radius, 1000, {0, radius, 0});
    floor.friction = ball.friction = mu;
    world.addBody(floor);
    const auto id = world.addBody(ball);
    for (int i = 0; i < 120; ++i) {
        world.step(h);
        if (world.report().stalled || world.report().unfinished != 0) return false;
    }
    Body& body = world.body(id);
    body.linearMomentum.x = speed / body.mass.inverseMass;
    world.step(h);
    if (world.report().stalled || world.report().unfinished != 0) return false;
    // I = 2mR²/5. Изменение скорости в контакте за импульс J: Δv = (1/m + R²/I) J.
    const Real dv = std::copysign(std::min(mu * gravity * h, Real(2) / 7 * std::abs(speed)), speed);
    const Real expectedCentre = speed - dv, expectedSlip = speed - Real(3.5) * dv;
    const Real centre = body.linearVelocity().x, slip = centre + radius * body.angularVelocity().z;
    const bool ok = std::abs(centre - expectedCentre) <= speedTolerance && std::abs(slip - expectedSlip) <= speedTolerance;
    std::printf("%.4g,%.4g,%.12g,%.12g,%.12g,%.12g,%d\n", double(mu), double(speed),
                double(expectedCentre), double(centre), double(expectedSlip), double(slip), int(ok));
    return ok;
}
}

int main() {
    if (const char* threads = std::getenv("KP_THREADS")) WorkerPool::shared().resize(std::atoi(threads));
    std::puts("mu,initial_m_s,oracle_centre_m_s,centre_m_s,oracle_slip_m_s,slip_m_s,pass");
    bool pass = true;
    for (Real mu : {Real(0), Real(0.5)})
        for (Real speed : {Real(-0.01), Real(0), Real(0.01), Real(0.1)}) pass &= measure(speed, mu);
    return pass ? 0 : 1;
}
