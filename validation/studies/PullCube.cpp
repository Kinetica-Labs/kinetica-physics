// Прибор «рывок мышью за зажатый куб»: привязь окна (8 Гц) за куб пирамиды из 55 (сцена pyramid55),
// рука идёт 0,5 с со скоростью speed м/с по направлению (dx, dy, dz), потом стоит. Мерки за 120 кадров:
// цена кадра, проходы решателя, перекрытие, среднее отставание куба от руки.
//   study-pull-cube [exact|realtime] [куб] [скорость, м/с] [dx dy dz]
// Худший случай: study-pull-cube realtime 14 2 1 0 0 — куб второго ряда вбок, в соседей.
#include "common.h"
#include <kp/physics.h>
#include <chrono>
#include <cstdlib>
using namespace kp;
int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "realtime";
    const BodyId held{argc > 2 ? std::atoi(argv[2]) : 14};
    const double speed = argc > 3 ? std::atof(argv[3]) : 2;
    const Vector3 along = argc > 6 ? Vector3{Real(std::atof(argv[4])), Real(std::atof(argv[5])), Real(std::atof(argv[6]))} : Vector3{1, 0, 0};
    SceneSpec spec;
    for (const SceneSpec& s : scenes()) if (std::string(s.key) == "pyramid55") spec = s;
    World world;
    world.realtime = mode == "realtime";
    for (const BoxSpec& b : spec.boxes) { Body body = Body::box({b.half.x, b.half.y, b.half.z}, b.density, {b.at.x, b.at.y, b.at.z}); body.orientation = {b.turn.w, b.turn.x, b.turn.y, b.turn.z}; world.addBody(body); }
    for (int f = 0; f < 60; ++f) world.step(Real(1) / 60); // успокоиться
    const Vector3 start = world.body(held).position;
    const double mass = 1 / double(world.body(held).mass.inverseMass), omega = 2 * M_PI * 8;
    TetherJoint& tether = world.addTether(BodyId(0), start, held, start, Real(mass * omega * omega), Real(2 * mass * omega));
    const int frames = 120;
    double wall = 0, worst = 0, overlap = 0, lag = 0; int iterations = 0, most = 0;
    for (int f = 0; f < frames; ++f) {
        const Vector3 hand = start + Real(speed * std::min((f + 1) / 60.0, 0.5)) * along;
        tether.anchorA = world.body(BodyId(0)).pose().localPoint(hand);
        const auto t0 = std::chrono::steady_clock::now();
        world.step(Real(1) / 60);
        const double ms = 1e3 * std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        wall += ms, worst = std::max(worst, ms), lag += double((hand - world.body(held).position).length());
        iterations += world.report().solverIterations, most = std::max(most, world.report().solverIterations);
        std::vector<State> now;
        for (const Body& x : world.bodies()) now.push_back({{double(x.position.x), double(x.position.y), double(x.position.z)}, {double(x.orientation.w), double(x.orientation.x), double(x.orientation.y), double(x.orientation.z)}});
        overlap = std::max(overlap, deepestOverlap(spec, now));
    }
    std::printf("%s, куб %d, рука %.1f м/с: %.1f мс/кадр средн., худший %.0f; проходов решателя %.1f средн., до %d; перекрытие до %.3f мм; отставание от руки %.3f м средн.\n",
                mode.c_str(), int(held), speed, wall / frames, worst, double(iterations) / frames, most, 1e3 * overlap, lag / frames);
}
