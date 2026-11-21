#pragma once
// Опыт «рывок куба, как в окне», общий для приборов (WindowPull.cpp, FrameCovariance.cpp): стандартная
// сцена с настройками окна — сон тел, сопротивление среды 0,1 1/с, привязь мыши 8 Гц — три секунды стоит,
// затем тело тянут рукой полсекунды со скоростью speed по направлению along и держат.
//
// Настройки шага для опытов с факторами берутся из окружения (stepSettings): KP_SUBSTEPS —
// SoftStep::substeps, KP_UPDATES — contactUpdates, KP_ITERATIONS — directIterations, KP_SEGMENTS —
// World::maxSegments (предел отрезков точного шага; в окне 8).

#include "../../examples/scenes/StandardScenes.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>

namespace kp {

inline void stepSettings(World& world) {
    if (const char* substeps = std::getenv("KP_SUBSTEPS")) world.soft.substeps = std::atoi(substeps);
    if (const char* updates = std::getenv("KP_UPDATES")) world.soft.contactUpdates = std::atoi(updates);
    if (const char* iterations = std::getenv("KP_ITERATIONS")) world.soft.directIterations = std::atoi(iterations);
    if (const char* segments = std::getenv("KP_SEGMENTS")) world.maxSegments = std::atoi(segments);
}

struct PullScenario {
    std::string mode = "realtime", key = "pyramid";
    int held = 12;
    double speed = 2;
    Vector3 along{1, 0, 0};

    // Аргументы прибора, начиная с argv[first]: [exact|realtime] [сцена] [тело] [скорость, м/с] [dx dy dz].
    static PullScenario parse(int argc, char** argv, int first) {
        PullScenario pull;
        if (argc > first) pull.mode = argv[first];
        if (argc > first + 1) pull.key = argv[first + 1];
        if (argc > first + 2) pull.held = std::atoi(argv[first + 2]);
        if (argc > first + 3) pull.speed = std::atof(argv[first + 3]);
        if (argc > first + 6) pull.along = {Real(std::atof(argv[first + 4])), Real(std::atof(argv[first + 5])), Real(std::atof(argv[first + 6]))};
        return pull;
    }

    // Мир сцены после трёх секунд покоя.
    World settled() const {
        World world;
        for (const StandardScene& scene : standardScenes())
            if (key == scene.key) world = scene.build();
        const bool exact = mode == "exact";
        world.sleeping = true, world.realtime = !exact;
        world.sleepSpeed = exact ? Real(1e-4) : Real(0.05), world.sleepTime = 2;
        stepSettings(world);
        for (size_t i = 0; i < world.bodies().size(); ++i) world.body(BodyId(int(i))).linearDamping = world.body(BodyId(int(i))).angularDamping = Real(0.1);
        for (int f = 0; f < 180; ++f) world.step(Real(1) / 60);
        return world;
    }

    struct Grip {
        TetherJoint* tether;
        Vector3 start; // где тело было в момент захвата
    };

    // Привязь окна к телу: пружина на 8 Гц с критическим затуханием.
    Grip grab(World& world) const {
        const BodyId body{held};
        world.wake(body);
        const Vector3 start = world.body(body).position;
        const double mass = 1 / double(world.body(body).mass.inverseMass), omega = 2 * M_PI * 8;
        return {&addTether(world, BodyId(0), start, body, start, Real(mass * omega * omega), Real(2 * mass * omega)), start};
    }

    // Положение руки перед кадром frame: полсекунды идёт, потом стоит.
    void drag(World& world, const Grip& grip, int frame) const {
        const Vector3 hand = grip.start + Real(speed * std::min((frame + 1) / 60.0, 0.5)) * along;
        grip.tether->anchorA = world.body(BodyId(0)).pose().localPoint(hand);
    }
};

} // namespace kp
