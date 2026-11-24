// Осевшая пара вёдер, верхнее поставлено со смещением на кромку. СИ, 60 кадров/с.
#include "OverlayComparison.h"
#include "../../viewer/ContactOverlay.h"
#include "../../examples/scenes/StandardScenes.h"
#include "../../examples/scenes/BucketRim.h"
#include <collision/PairFinder.h>
#include <math/WorkerPool.h>
#include <array>
#include <cmath>
#include <cstdio>
#include <string_view>

namespace {
using namespace kp;
constexpr Real dt = Real(1) / 60;
constexpr BodyId top{3};

using Poses = std::array<Transform, 3>;
Poses poses(const World& world) {
    return {world.body(BodyId{1}).pose(), world.body(BodyId{2}).pose(), world.body(top).pose()};
}

double surfaceTravel(const World& world, const Poses& first, const Poses& now) {
    double maximum = 0;
    for (size_t i = 0; i < now.size(); ++i) {
        const auto q = now[i].orientation * first[i].orientation.conjugate();
        const Real radius = world.bodies()[i + 1].boundingRadius();
        const Real chord = 2 * radius * Vector3{q.x, q.y, q.z}.length();
        maximum = std::max(maximum, double((now[i].position - first[i].position).length() + chord));
    }
    return maximum;
}

bool hold(const StandardScene& scene) {
    World world = bucket_rim::releaseOnRim(scene.build());
    constexpr int first = 47 * 60, last = 57 * 60;
    // Независимый образ нижней опоры для проверки всех отметок после отрыва.
    World lower;
    for (int i = 0; i < int(top); ++i) lower.addBody(world.body(BodyId{i}));
    ContactOverlay overlay, lowerOverlay;
    Poses start{};
    DebugGeometry previousRaw, previousShown;
    double maxTravel = 0, maxSpeed = 0, maxGain = 0, priorEnergy = 0;
    int rawChanges = 0, shownChanges = 0;
    bool tilted = true;
    std::puts("frame,tilt_deg,travel_m,surface_speed_m_s,kinetic_J,potential_J,raw_points,shown_points,raw_change,shown_change");
    for (int frame = 1; frame <= last; ++frame) {
        world.step(dt);
        if (world.report().stalled || world.report().unfinished != 0) return false;
        if (frame < first) continue;
        const auto now = poses(world);
        if (frame == first) start = now;
        const double travel = surfaceTravel(world, start, now);
        double kinetic = 0, potential = 0, speed = 0;
        for (const auto& b : world.bodies()) if (!b.isStatic()) {
            kinetic += kineticEnergyOf(b);
            potential -= world.gravity.dot(b.position) / b.mass.inverseMass;
            speed = std::max(speed, double(b.linearVelocity().length() + b.boundingRadius() * b.angularVelocity().length()));
        }
        if (!std::isfinite(travel + kinetic + potential + speed)) return false;
        DebugGeometry raw;
        raw.addContactMarkers(world);
        const DebugGeometry& shown = overlay.sample(world);
        for (int i = 0; i < int(top); ++i) lower.body(BodyId{i}) = world.body(BodyId{i});
        lowerOverlay.sample(lower);
        const bool rawChange = frame > first && overlay_test::visiblyDifferent(raw.lines, previousRaw.lines);
        const bool shownChange = frame > first && overlay_test::visiblyDifferent(shown.lines, previousShown.lines);
        rawChanges += rawChange;
        shownChanges += shownChange;
        maxTravel = std::max(maxTravel, travel);
        maxSpeed = std::max(maxSpeed, speed);
        if (frame > first) maxGain = std::max(maxGain, kinetic + potential - priorEnergy);
        priorEnergy = kinetic + potential;
        const auto up = world.body(top).orientation.rotate({0, 1, 0});
        const double tilt = std::atan2(std::hypot(double(up.x), double(up.z)), double(up.y)) * 180 / double(kPi);
        tilted = tilted && tilt > 25 && tilt < 40;
        std::printf("%d,%.17g,%.17g,%.17g,%.17g,%.17g,%zu,%zu,%d,%d\n",
                    frame, tilt, travel, speed, kinetic, potential, raw.points.size(), shown.points.size(), rawChange, shownChange);
        previousRaw = std::move(raw);
        previousShown = shown;
    }
    const bool held = bucket_rim::topTouches(world);
    world.body(top).position.y += 1;
    const bool detached = !bucket_rim::topTouches(world);
    const auto& shown = overlay.sample(world);
    const bool released = detached && shown.points.size() < previousShown.points.size() &&
                          !shown.points.empty() && overlay_test::exact(shown, lowerOverlay.sample(lower));
    std::fprintf(stderr, "intervals=%d raw_changes=%d shown_changes=%d max_travel_m=%.12g max_speed_m_s=%.12g "
                         "max_frame_gain_J=%.12g held=%d released=%d\n",
                 last-first, rawChanges, shownChanges, maxTravel, maxSpeed, maxGain, held, released);
    return tilted && held && released && maxTravel < 1e-6 && maxSpeed < 1e-5 && maxGain < 1e-8 && shownChanges == 0;
}
} // namespace

int main() {
    kp::WorkerPool::shared().resize(1);
    for (const auto& scene : standardScenes())
        if (std::string_view(scene.key) == "buckets") return hold(scene) ? 0 : 1;
    return 2;
}
