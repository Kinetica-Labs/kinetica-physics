// Наш движок на общих сценах (common.h).
//
//   compare-ours [realtime ПОДШАГОВ | exact] [sleep] [--shock-on|--shock-off]
//   KP_THREADS=1 — число потоков пула: Box3D в сравнении считает одним потоком
#include "stress/Stress1000.h"
#include <physics.h>
#include <math/WorkerPool.h>

#include <cstdlib>

using namespace kp;

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "realtime";
    const int substeps = argc > 2 ? std::atoi(argv[2]) : 8;
    bool sleep = false, support = World::kDefaultShockPropagation;
    for (int i = mode == "exact" ? 2 : 3; i < argc; ++i) {
        const std::string option = argv[i];
        if (option == "sleep") sleep = true;
        else if (option == "--shock-on") support = true;
        else if (option == "--shock-off") support = false;
        else { std::fprintf(stderr, "unknown option: %s\n", argv[i]); return 2; }
    }
    if (const char* threads = std::getenv("KP_THREADS")) WorkerPool::shared().resize(std::atoi(threads));
    const std::string engine = mode == "exact" ? "kinetica exact" : "kinetica realtime " + std::to_string(substeps) + (sleep ? " sleep" : "");
    const bool stressRun = stress::requested();
    if (stressRun) stress::require(mode == "realtime" && substeps == 1 && !sleep, "stress requires realtime 1, sleep off");
    stress::compareOrStress(engine, [&](const SceneSpec& s, const Play& play) {
        World world;
        world.realtime = mode == "realtime";
        world.shockPropagation = support;
        world.soft.substeps = substeps;
        if (stressRun) {
            world.soft.contactUpdates = 1;
            world.soft.coupledFriction = world.soft.reuseEquilibrium = true;
        }
        world.maxSegments = 64;
        world.sleeping = sleep, world.sleepSpeed = Real(0.05); // порог окна, как у Box3D
        for (const BoxSpec& b : s.boxes) {
            Body body = Body::box({b.half.x, b.half.y, b.half.z}, b.density, {b.at.x, b.at.y, b.at.z});
            body.orientation = {b.turn.w, b.turn.x, b.turn.y, b.turn.z};
            if (b.density != 0) body.linearMomentum = (1 / body.mass.inverseMass) * Vector3{b.velocity.x, b.velocity.y, b.velocity.z};
            world.addBody(body);
        }
        for (const JointSpec& j : s.joints) {
            const Vector3 at{j.anchor.x, j.anchor.y, j.anchor.z};
            if (j.weld) addWeld(world, BodyId(j.a), BodyId(j.b), at);
            else addHinge(world, BodyId(j.a), BodyId(j.b), at, {0, 0, 1});
        }
        play([&](double dt) {
            world.step(Real(dt));
            if (stressRun) stress::require(!world.report().stalled && world.report().unfinished == 0, "unfinished Kinetica step");
        }, [&] {
            std::vector<State> st;
            for (size_t i = 0; i < world.bodies().size(); ++i) {
                const Body& b = world.bodies()[i];
                const Vector3 v = b.linearVelocity(), w = b.angularVelocity();
                st.push_back({{b.position.x, b.position.y, b.position.z}, {b.orientation.w, b.orientation.x, b.orientation.y, b.orientation.z}, {v.x, v.y, v.z}, {w.x, w.y, w.z}, world.asleep(BodyId(int(i)))});
            }
            return st; });
    });
}
