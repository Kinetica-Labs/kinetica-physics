#pragma once
// Одна и та же траектория захвата при добавлении механически независимых тел.
#include <world/World.h>
#include <array>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <optional>

namespace grip {
enum class Support { floor, weld, spring };
struct Neighbour { const char* name; kp::Real x, gap; Support support = Support::floor; };

inline std::optional<kp::BodyId> addNeighbour(kp::World& world, kp::BodyId base, Neighbour scene) {
    if (scene.x == 0) return std::nullopt;
    const kp::Vector3 at{scene.x, kp::Real(0.1) + scene.gap, 0};
    if (scene.support == Support::weld) {
        kp::Body anchor;
        anchor.position = at;
        kp::addWeld(world, base, world.addBody(anchor), at);
        return std::nullopt;
    }
    if (scene.support == Support::floor)
        world.addBody(kp::Body::box({1, kp::Real(0.1), 1}, 0, {scene.x, kp::Real(-0.1), 0}));
    // Объём 0,008 м³ и плотность 125 кг/м³ дают 1 кг. Сила направлена только на этот куб.
    const auto cube = world.addBody(kp::Body::box({kp::Real(0.1), kp::Real(0.1), kp::Real(0.1)}, 125, at));
    if (scene.support == Support::spring) {
        auto& slider = kp::addSlider(world, base, cube, at, {0,1,0});
        slider.stiffness = 100;
        slider.restPosition = kp::Real(0.1); // F_y = −k (0 − 0,1 м) = +10 Н в начальном равновесии
    }
    return cube;
}

template<class MakeRig> int checkIslandIndependence(MakeRig makeRig, std::initializer_list<Neighbour> scenes) {
    constexpr kp::Real h = kp::Real(1) / 480;
    constexpr int steps = 120;
    constexpr double pathToleranceMm = 0.001, impulseToleranceNs = 1e-6;
    std::array<double, steps> reference{};
    bool pass = true;
    std::puts("scene,microstep,drop_mm,reference_mm,difference_mm,remote_up_Ns");
    for (Neighbour scene : scenes) {
        auto rig = makeRig();
        auto& world = rig.world();
        const auto id = addNeighbour(world, rig.anchor(), scene);
        double initialY = 0;
        for (int step = -steps; step < steps; ++step) {
            if (step == 0) initialY = rig.read().relativeY;
            const kp::Real before = id ? world.body(*id).linearMomentum.y : 0;
            if (id) world.body(*id).force.y = -10;
            if (!rig.advance(h, 10, step < 0 ? 0 : -10)) return 2;
            if (step < 0) continue;
            const double drop = 1000 * (initialY - rig.read().relativeY);
            if (scene.x == 0) reference[step] = drop;
            const double diff = drop - reference[step];
            // Независимая проверка, что удалённая опора действительно приняла вертикальный импульс.
            const double impulse = id ? world.body(*id).linearMomentum.y - before + 10 * h : 0;
            const bool supported = scene.support == Support::spring || (id && scene.gap == 0);
            const double expected = supported ? 10 * h : 0;
            pass &= std::abs(diff) <= pathToleranceMm && std::abs(impulse - expected) <= impulseToleranceNs;
            std::printf("%s,%d,%.12g,%.12g,%.12g,%.12g\n", scene.name, step + 1, drop, reference[step], diff, impulse);
        }
        std::fflush(stdout); // сохраняет предшествующую сцену даже при отказе на следующей
    }
    return pass ? 0 : 1;
}
}
