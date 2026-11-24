// Независимые простые оракулы для прохода по опорам и целого World::step.
#include "SupportAudit.h"
#include "BeamScene.h"
#include <physics.h>
#include <math/WorkerPool.h>
#include <cmath>
#include <cstdio>

using namespace kp;
namespace {
constexpr Real h = Real(1) / 480;

void result(const char* name, Real error, Real tolerance, const char* unit) {
    std::printf("%s,%.12g,%.12g,%s,%d\n", name, error, tolerance, unit, int(std::isfinite(error) && error<=tolerance));
}

struct Fixture {
    std::vector<Body> bodies;
    StepArray<SolverBody> states;
    StepArray<SoftContact> contacts;

    Fixture(bool fixed, Vector3 normal, Vector3 at = {}, Real gap = 0) {
        bodies = {Body::box({.5,.5,.5}, fixed ? 0 : 1, -.5*normal),
                  Body::box({.5,.5,.5}, 1, .5*normal)};
        for (const Body& body : bodies) {
            auto& s = states.emplace_back();
            s.inverseMass = body.mass.inverseMass;
            if (!body.isStatic()) s.inverseInertia = inverseInertiaInWorld(body);
        }
        ContactPair pair;
        pair.a = 0; pair.b = 1; pair.manifold.normal = normal;
        pair.manifold.add({at, at, gap, 1});
        contacts.emplace_back(pair, bodies, states, Softness::rigid(), Span<ContactForces::Entry>{}, h, false, false);
    }
    void project() {
        support_study::project(contacts[0], 0, states, h, [](const auto&, Real, auto push) { push(); });
    }
    auto totals() const {
        support_study::Totals t;
        for (size_t i=0; i<bodies.size(); ++i) t.add(bodies[i], states[i]);
        return t;
    }
};

void analytic() {
    const StepMemory memory;
    Fixture slope(true, {.6,.8,0});
    slope.states[1].v = {0,-1,0};
    const auto before = slope.totals();
    slope.project();
    const auto after = slope.totals();
    result("incline_reaction", (after.momentum-before.momentum-Vector3{.48,.64,0}).length(), 1e-9,"Ns");
    result("incline_dissipation", kp::abs(after.kinetic-before.kinetic + .32), 1e-9,"J");

    Fixture two(false, {0,1,0});
    two.states[1].v = {0,-1,0};
    const auto floating = support_study::orderOf(two.contacts, two.bodies, {0,-9.81,0});
    result("floating_component_edges", Real(floating.size()), 0,"count");
    two.project();
    result("equal_masses_shared_velocity", std::max(kp::abs(two.states[0].v.y+.5),kp::abs(two.states[1].v.y+.5)), 1e-9,"m/s");

    Fixture eccentric(false, {0,1,0}, {.23,0,.1});
    eccentric.states[1].v = {.1,-1,.2};
    const auto start = eccentric.totals();
    eccentric.project();
    const auto end = eccentric.totals();
    result("eccentric_delta_P", (end.momentum-start.momentum).length(), 1e-9,"Ns");
    result("eccentric_delta_L", (end.angular-start.angular).length(), 1e-9,"Nms");
    result("eccentric_energy_gain", std::max(Real(0),end.kinetic-start.kinetic), 1e-9,"J");

    Fixture separating(true,{0,1,0});
    separating.states[1].v = {0,1,0};
    separating.project();
    result("separating_impulse", separating.contacts[0].normalRow(0).impulse, 0,"Ns");
    Fixture gap(true,{0,1,0},{},.001);
    gap.states[1].v = {0,-.1,0}; // путь 0,208 мм, зазор 1 мм
    gap.project();
    result("open_gap_impulse", gap.contacts[0].normalRow(0).impulse, 0,"Ns");
    result("zero_gravity_edges", Real(support_study::orderOf(slope.contacts,slope.bodies,{}).size()), 0,"count");
}

// Доска 1,3 м лежит на опоре шириной 25 мм. Центры грузов в ±0,45 м,
// 1 кг при плотности 125 кг/м³. Угол положителен, когда поднимается правый край.
Real beam(Real left, Real right, Real& peak) {
    World world = support_scene::beam(left, right);
    world.realtime = true;
    world.sleeping = false;
    world.soft.substeps = world.soft.contactUpdates = 1;
    Real angle = 0;
    for (int i=0; i<120; ++i) {
        world.step(h);
        const auto x = world.bodies()[1].orientation.rotate({1,0,0});
        angle = std::atan2(x.y,x.x) * 180 / kPi;
        peak = std::max(peak,kp::abs(angle));
    }
    return angle;
}
}

int main() {
    WorkerPool::shared().resize(1);
    std::puts("case,observed,upper_limit,unit,pass");
    analytic();
    Real equalPeak=0, leftPeak=0, rightPeak=0;
    beam(1,1,equalPeak);
    const Real left=beam(2,1,leftPeak), right=beam(1,2,rightPeak);
    result("balanced_beam_peak",equalPeak,.5,"deg");
    result("left_heavier_negative_of_angle",-left,-1,"deg");
    result("right_heavier_angle",right,-1,"deg");
    result("mirror_angle_sum",kp::abs(left+right),.5,"deg");
}
