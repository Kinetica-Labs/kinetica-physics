// Баланс энергии трёх вёдер при свободном падении и покое. Отсчёт с первого кадра, СИ.
#include "../../../examples/scenes/StandardScenes.h"
#include <math/WorkerPool.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string_view>

namespace {
using namespace kp;

struct Reading {
    double kinetic = 0, potential = 0, pointSpeed = 0;
};

Reading read(const World& world) {
    Reading r;
    for (const auto& b : world.bodies()) {
        if (b.isStatic()) continue;
        r.kinetic += double(kineticEnergyOf(b));
        r.potential -= double(world.gravity.dot(b.position)) / b.mass.inverseMass;
        r.pointSpeed = std::max(r.pointSpeed, double(b.linearVelocity().length() + b.boundingRadius() * b.angularVelocity().length()));
    }
    return r;
}

bool record(const StandardScene& scene, double degrees, double offset) {
    World world = scene.build();
    world.realtime = true;
    world.sleeping = false;
    auto& top = world.body(BodyId{3});
    top.orientation = Quaternion::fromRotationVector({0, 0, Real(degrees) * kPi / 180});
    top.position.x += Real(offset);
    for (int i = 0; i < int(world.bodies().size()); ++i) {
        auto& body = world.body(BodyId{i});
        body.linearDamping = body.angularDamping = 0;
    }
    for (int i = 0; i <= 86 * 60; ++i) {
        if (i > 0) world.step(Real(1) / 60);
        const auto r = read(world);
        if (!std::isfinite(r.kinetic + r.potential) || world.report().stalled || world.report().unfinished != 0) return false;
        const auto up = top.orientation.rotate({0,1,0});
        const double tilt = std::atan2(std::hypot(double(up.x), double(up.z)), double(up.y)) * 180.0 / double(kPi);
        std::printf("%.0f,%.3f,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n",
                    degrees, offset, i, r.kinetic, r.potential, r.pointSpeed, tilt,
                    double(top.position.x), double(top.position.y), double(top.position.z));
    }
    return true;
}
} // namespace

int main() {
    kp::WorkerPool::shared().resize(1);
    std::puts("initial_tilt_deg,initial_offset_m,frame,kinetic_J,potential_J,point_speed_bound_m_s,top_tilt_deg,top_x_m,top_y_m,top_z_m");
    for (const auto& scene : standardScenes()) {
        if (std::string_view(scene.key) == "buckets")
            return record(scene, 0, 0) && record(scene, 16, -0.045) && record(scene, -16, 0.045) ? 0 : 1;
    }
    return 2;
}
