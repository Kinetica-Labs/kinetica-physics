// Проверка World::step по независимым траекториям и геометрии шарниров.
#include <physics.h>
#include <math/WorkerPool.h>
#include "ExactScenes.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

using namespace kp;
namespace {
struct Error {
    double limit, worst = 0;
    int firstFailure = 0, peakFrame = 0;

    void observe(double value, int frame) {
        value = std::isfinite(value) ? std::abs(value) : std::numeric_limits<double>::infinity();
        if (value > worst) { worst = value; peakFrame = frame; }
        if (value > limit && !firstFailure) firstFailure = frame;
    }
};

struct Run {
    int rate;
    bool realtime, support;

    void configure(World& world) const {
        world.realtime = realtime;
        world.shockPropagation = support;
        world.sleeping = false;
        world.maxSegments = 64;
    }
    void step(World& world, int frame, int& unfinished) const {
        world.step(Real(1) / rate);
        if (!unfinished && (world.report().stalled || world.report().unfinished != 0)) unfinished = frame;
    }
    void record(const char* scene, const char* metric, const char* unit, const Error& e, int unfinished) const {
        std::printf("%s,%s,%d,%d,%s,%s,%.12g,%.12g,%d,%d,%d,%d,%d\n", scene, realtime ? "realtime" : "event",
                    int(support), rate, metric, unit, e.worst, e.limit, int(!unfinished), int(!unfinished && !e.firstFailure),
                    e.firstFailure, e.peakFrame, unfinished);
    }
};

void flight(const Run& run) {
    World world;
    run.configure(world);
    world.addBody(Body::sphere(.1, 1000, {0,2,0}));
    Error position{.0005}, velocity{1e-9};
    int unfinished = 0;
    for (int i = 1; i <= run.rate / 2; ++i) {
        run.step(world, i, unfinished);
        const double t = double(i) / run.rate, g = -9.81;
        const auto& b = world.bodies()[0];
        position.observe(b.position.y - (2 + g*t*t/2), i);
        velocity.observe(b.linearVelocity().y - g*t, i);
    }
    run.record("free_fall", "max_y", "m", position, unfinished);
    run.record("free_fall", "max_vy", "m/s", velocity, unfinished);
}

Vector3 angular(const Body& b) { return b.position.cross(b.linearMomentum) + b.angularMomentum; }

void impact(const Run& run) {
    World world;
    run.configure(world);
    world.gravity = {};
    constexpr Real radius = .1, offset = .09, diameter = 2 * radius;
    auto a = Body::sphere(radius, 1000, {-.25,0,-offset/2});
    auto b = Body::sphere(radius, 1000, { .25,0, offset/2});
    a.restitution = b.restitution = 1;
    a.friction = b.friction = 0;
    a.linearMomentum = {1 / a.mass.inverseMass, 0, 0};
    b.linearMomentum = {-.5 / b.mass.inverseMass, 0, 0};
    world.addBody(a);
    world.addBody(b);
    // У равных гладких сфер при e=1 меняются местами нормальные компоненты скоростей.
    const Real contactX = std::sqrt(diameter*diameter - offset*offset);
    const Vector3 normal{contactX / diameter, 0, offset / diameter};
    const Vector3 transfer = normal.dot(a.linearVelocity() - b.linearVelocity()) * normal;
    const auto& bodies = world.bodies();
    const Real energy = kineticEnergyOf(a) + kineticEnergyOf(b);
    Error dp{1e-9}, dl{1e-9}, de{1e-8}, dv{.01};
    int unfinished = 0;
    for (int i = 1; i <= run.rate / 2; ++i) {
        run.step(world, i, unfinished);
        dp.observe((bodies[0].linearMomentum + bodies[1].linearMomentum - (a.linearMomentum + b.linearMomentum)).length(), i);
        dl.observe((angular(bodies[0]) + angular(bodies[1]) - (angular(a) + angular(b))).length(), i);
        de.observe(kineticEnergyOf(bodies[0]) + kineticEnergyOf(bodies[1]) - energy, i);
    }
    dv.observe((bodies[0].linearVelocity() - (a.linearVelocity() - transfer)).length(), run.rate / 2);
    dv.observe((bodies[1].linearVelocity() - (b.linearVelocity() + transfer)).length(), run.rate / 2);
    run.record("elastic_oblique", "post_velocity", "m/s", dv, unfinished);
    run.record("elastic_oblique", "max_delta_P", "N.s", dp, unfinished);
    run.record("elastic_oblique", "max_delta_L", "N.m.s", dl, unfinished);
    run.record("elastic_oblique", "max_delta_T", "J", de, unfinished);
}

// Геометрический прибор по двум мировым позам. Шаровая опора допускает любой поворот,
// у ползуна перемещение вдоль оси свободно. Угол вычисляем через atan2 в радианах.
void measureJoint(const World& world, const Joint& joint, int frame, Error& gap, Error& angle, bool& hasAngle) {
    const auto& a = world.bodies()[joint.a];
    const auto& b = world.bodies()[joint.b];
    const Vector3 apart = b.pose().point(joint.anchorB) - a.pose().point(joint.anchorA);
    const Quaternion qa = a.orientation * joint.frameA, qb = b.orientation * joint.frameB;
    const Vector3 axis = qa.rotate({1,0,0});
    const bool slider = dynamic_cast<const SliderJoint*>(&joint) != nullptr;
    gap.observe(slider ? apart.cross(axis).length() : apart.length(), frame);
    if (dynamic_cast<const BallAndSocketJoint*>(&joint)) return;
    hasAngle = true;
    if (dynamic_cast<const HingeJoint*>(&joint)) {
        const Vector3 other = qb.rotate({1,0,0});
        angle.observe(std::atan2(axis.cross(other).length(), axis.dot(other)), frame);
    } else {
        const Quaternion relative = qa.conjugate() * qb; // ползун или сварка
        angle.observe(2 * std::atan2(Vector3{relative.x,relative.y,relative.z}.length(), std::abs(relative.w)), frame);
    }
}

void mechanism(const Run& run, const ExactScene& scene) {
    World world = scene.build();
    run.configure(world);
    const bool pendulum = std::strcmp(scene.key, "pendulum") == 0;
    const bool chain = std::strcmp(scene.key, "chain100") == 0;
    const double limit = chain ? .001 * std::abs(scene.exact(0)) : (pendulum ? .001 : .0005);
    Error coordinate{limit}, gap{.0005}, angle{.001};
    bool hasAngle = false;
    int unfinished = 0;
    for (int i = 1; i <= run.rate; ++i) {
        run.step(world, i, unfinished);
        coordinate.observe(scene.measured(world) - scene.exact(double(i) / run.rate), i);
        for (const Joint& joint : world.joints()) measureJoint(world, joint, i, gap, angle, hasAngle);
    }
    run.record(scene.key, chain ? "max_energy_change" : "max_coordinate", chain ? "J" : pendulum ? "rad" : "m", coordinate, unfinished);
    run.record(scene.key, "max_anchor_gap", "m", gap, unfinished);
    if (hasAngle) run.record(scene.key, "max_orientation_error", "rad", angle, unfinished);
}
} // namespace

int main() {
    WorkerPool::shared().resize(1);
    std::puts("scene,solver,support,Hz,metric,unit,error,limit,complete,pass,first_failure_frame,peak_frame,first_unfinished_frame");
    for (int rate : {60,120,240}) for (bool realtime : {false,true}) for (bool support : {false,true}) {
        const Run run{rate, realtime, support};
        flight(run);
        impact(run);
        for (const auto& scene : exactScenes()) mechanism(run, scene);
    }
}
