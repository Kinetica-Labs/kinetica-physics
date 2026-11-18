// Прибор: куб падает на пол углом вперёд (сцена 4 окна просмотра). По кадрам печатаются зазор
// до пола, энергия и работа шага. При нулевом коэффициенте восстановления энергия обязана только
// убывать, и только в моменты ударов; между ударами — сохраняться.

#include <kp/collision/PairFinder.h>
#include <kp/collision/ContactPair.h>
#include <kp/world/World.h>

#include <algorithm>
#include <cstdio>

using namespace kp;

// Учёт энергии по стадиям шага: после каждой стадии записывается, насколько изменилась энергия.
struct EnergyLedger final : StepObserver {
    Real previous = 0, gained[4] = {}, lost[4] = {};
    bool print = false;
    Real (*energyOf)(const World&) = nullptr;
    void after(StepStage stage, const World& world) override {
        const Real now = energyOf(world), change = now - previous;
        (change > 0 ? gained : lost)[int(stage)] += change;
        if (print) std::printf("        стадия %d: энергия %.6f, изменение %+.3e\n", int(stage), now, change);
        previous = now;
    }
};

Real cubeEnergy(const World& world) {
    const Body& b = world.bodies()[1];
    return b.kineticEnergy() + Real(9.81) * b.position.y / b.mass.inverseMass;
}

int main() {
    World world;
    world.addBody(Body::box({5, 0.5, 5}, 0, {0, -0.5, 0}));
    Body cube = Body::box({0.3, 0.3, 0.3}, 1000, {0, 2, 0});
    cube.orientation = Quaternion{1, Real(0.3), Real(0.2), Real(0.4)}.unit();
    const BodyId id = world.addBody(cube);
    const ConvexHull corners = ConvexHull::box({0.3, 0.3, 0.3});

    const auto energy = [&] {
        const Body& b = world.body(id);
        return b.kineticEnergy() + Real(9.81) * b.position.y / b.mass.inverseMass;
    };
    const auto gaps = [&](Real& lowest, int& near) { // зазор нижней вершины и число вершин ближе 1 мм к полу
        lowest = 1e9, near = 0;
        for (Vector3 v : corners.vertices()) {
            const Real y = world.body(id).pose().point(v).y;
            lowest = std::min(lowest, y);
            near += y < 1e-3;
        }
    };

    std::printf("%5s %9s %11s %5s %12s %12s %6s %6s %5s\n", "кадр", "t, с", "зазор, мм", "вершин", "энергия, Дж", "Δэнергии", "контакт", "отрезк", "проход");
    Real previous = energy(), largestGain = 0;
    EnergyLedger ledger;
    ledger.energyOf = cubeEnergy;
    ledger.previous = previous;
    world.observer = &ledger;
    for (int frame = 1; frame <= 180; ++frame) {
        ledger.print = frame == 100;
        world.step(Real(1) / 60);
        Real lowest;
        int near, contacts = 0;
        gaps(lowest, near);
        for (const ContactPair& pair : PairFinder(world.bodies(), world.gravity).pairs(0))
            if (pair.inContact()) contacts += pair.manifold.count;
        const Real now = energy();
        largestGain = std::max(largestGain, now - previous);
        if (frame >= 30 && (frame <= 100 || frame % 10 == 0))
            std::printf("%5d %9.3f %11.4f %5d %12.4f %+12.2e %6d %6d %5d\n", frame, frame / 60.0, 1e3 * lowest, near, now, now - previous,
                        contacts, world.report().segments, world.report().solverIterations);
        previous = now;
    }
    std::printf("наибольший прирост энергии за кадр: %.3e Дж\n", largestGain);
    const char* names[] = {"раздвигание", "удары", "силы", "движение"};
    for (int s = 0; s < 4; ++s) std::printf("стадия «%s»: прибавила %.4f Дж, отняла %.4f Дж\n", names[s], ledger.gained[s], -ledger.lost[s]);
    return 0;
}
