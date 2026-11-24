#include "Scenes.h"
#include "HullEventsScene.h"
#include "ThresholdGrip.h"
#include "GlRenderer.h"
#include "../examples/scenes/CubeLayout.h"
#include "../examples/scenes/BeamScene.h"

#include "../examples/scenes/FloatingBox.h"
#include "../examples/scenes/GearTrain.h"
#include "../examples/scenes/RackDrive.h"
#include "../examples/scenes/RigidRod.h"
#include "../examples/scenes/WheelBench.h"
#include "../examples/scenes/Vehicle.h"
#include "../examples/scenes/StandardScenes.h"
#include <collision/shapes/Cylinder.h>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <string_view>

using namespace kp;

namespace {

World stressCubes(bool rain) {
    World world;
    world.soft.coupledFriction = world.soft.reuseEquilibrium = true;
    world.addBody(Body::box({20,.25,20}, 0, {0,-.25,0}));
    const Real a = Real(stress::halfSide);
    stress::forEachCube(rain, [&](double x, double y, double z, double vx, double vz) {
        Body cube = Body::box({a,a,a}, 64, {Real(x),Real(y),Real(z)});
        cube.linearMomentum = (1 / cube.mass.inverseMass) * Vector3{Real(vx),0,Real(vz)};
        world.addBody(cube);
    });
    return world;
}

World withFloor() {
    World world;
    world.addBody(Body::box({5, 0.5, 5}, 0, {0, -0.5, 0}));
    return world;
}

// Шар летит со скоростью 5 м/с в стопку из трёх кубов.
World ballHitsStack() {
    World world = withFloor();
    for (int k = 0; k < 3; ++k) world.addBody(Body::box({0.2, 0.2, 0.2}, 1000, {0, 0.2 + 0.4 * k, 0}));
    Body ball = Body::sphere(0.15, 1000, {-2, 0.9, 0});
    ball.linearMomentum = {5 / ball.mass.inverseMass, 0, 0};
    world.addBody(ball);
    return world;
}

// Мяч с коэффициентом восстановления 0,8: каждый отскок — 0,64 высоты предыдущего.
World bouncingBall() {
    World world;
    Body floor = Body::box({5, 0.5, 5}, 0, {0, -0.5, 0});
    Body ball = Body::sphere(0.2, 1000, {0, 2, 0});
    floor.restitution = ball.restitution = Real(0.8);
    world.addBody(floor);
    world.addBody(ball);
    return world;
}

World stackOfTen() {
    World world = withFloor();
    for (int k = 0; k < 10; ++k) world.addBody(Body::box({0.2, 0.2, 0.2}, 1000, {0, 0.2 + 0.4 * k, 0}));
    return world;
}

// Куб падает на пол углом вперёд.
World cubeOnCorner() {
    World world = withFloor();
    Body cube = Body::box({0.3, 0.3, 0.3}, 1000, {0, 2, 0});
    cube.orientation = Quaternion{1, Real(0.3), Real(0.2), Real(0.4)}.unit();
    world.addBody(cube);
    return world;
}

// Выпуклые оболочки случайных облаков точек и двенадцатигранная призма падают на пол.
World fallingHulls() {
    World world = withFloor();
    unsigned long long random = 2026;
    const auto uniform = [&] {
        random = random * 6364136223846793005ull + 1442695040888963407ull;
        return Real(random >> 11) / Real(1ull << 53);
    };
    for (int k = 0; k < 5; ++k) {
        std::vector<Vector3> cloud;
        for (int i = 0; i < 6 + 8 * k; ++i) cloud.push_back({Real(0.5) * uniform() - Real(0.25), Real(0.4) * uniform() - Real(0.2), Real(0.5) * uniform() - Real(0.25)});
        world.addBody(Body::hull(cloud, 1000, {Real(0.9) * k - Real(1.8), Real(1) + Real(0.3) * k, 0}));
    }
    std::vector<Vector3> prism;
    Real x = Real(0.2), z = 0;
    for (int k = 0; k < 12; ++k) {
        prism.push_back({x, Real(-0.3), z}), prism.push_back({x, Real(0.3), z});
        const Real next = Real(0.86602540378443865) * x - Real(0.5) * z;
        z = Real(0.5) * x + Real(0.86602540378443865) * z, x = next;
    }
    Body roller = Body::hull(prism, 1000, {0, 3, 1});
    roller.orientation = Quaternion{1, 0, 0, Real(0.9)}.unit(); // почти на боку: покатится
    world.addBody(roller);
    return world;
}

// Капсулы и цилиндры: один цилиндр стоит, другой катится, остальные падают как придётся.
World roundShapes() {
    World world = withFloor();
    world.addBody(Body::cylinder(0.3, 0.2, 1000, {-1.5, Real(0.30005), 0}));
    Body roller = Body::cylinder(0.25, 0.2, 1000, {-3, Real(0.20005), 1.2});
    roller.orientation = Quaternion{1, 1, 0, 0}.unit();
    // Катится без проскальзывания со скоростью 0,6 м/с: ω = v / r вокруг своей оси, L = I ω.
    roller.linearMomentum = {Real(0.6) / roller.mass.inverseMass, 0, 0};
    roller.angularMomentum = {0, 0, Real(-0.6) / Real(0.2) / roller.mass.inverseInertia.y};
    world.addBody(roller);
    for (int k = 0; k < 4; ++k) {
        Body body = k % 2 ? Body::capsule(0.25, 0.1, 1000) : Body::cylinder(0.2, 0.15, 1000, {}, Real(0.01));
        body.position = {Real(0.8) * k - Real(0.5), Real(1) + Real(0.4) * k, Real(-0.6)};
        body.orientation = Quaternion{1, Real(0.3) * k, Real(0.5), Real(-0.2) * k}.unit();
        world.addBody(body);
    }
    return world;
}

// Кубы одним столбиком: каждый висит в сантиметре над предыдущим, нижний — в сантиметре над
// полом. Сцена начинается с того, что кубы садятся друг на друга снизу вверх.
World column(int count) {
    World world = withFloor();
    const Real half = Real(0.1), gap = Real(0.01);
    for (int k = 0; k < count; ++k) world.addBody(Body::box({half, half, half}, 1000, {0, half + gap + (2 * half + gap) * k, 0}));
    return world;
}
World columnOfFifty() { return column(50); }
// Пирамида из 55 кубов: десять рядов, в нижнем десять кубов.
World pyramidOfFiftyFive() {
    World world = withFloor();
    for (int row = 0; row < 10; ++row)
        for (int k = 0; k < 10 - row; ++k) world.addBody(Body::box({0.2, 0.2, 0.2}, 1000, {Real(0.4) * (Real(k) - Real(9 - row) / 2), Real(0.2) + Real(0.4) * row, 0}));
    return world;
}
World columnOfFourHundred() { return column(400); }

// Первый контрольный кадр caps-at-rest: sharp_rim_capward касается rounded_cap без зазора.
const Transform rimFrame{{-4, 2, 3}, Quaternion{1, -2, 3, 5}.unit()};
const Vector3 rimAnchor = rimFrame.point({.125, -.25, .375});

World cylinderRim() {
    const Vector3 axis{.36, .48, .8}, radial{.8, 0, .6}, up{0, 1, 0};
    const Vector3 normal = .6 * radial + .8 * up;
    const auto align = [](Vector3 from, Vector3 to) {
        const Vector3 v = from.cross(to);
        return Quaternion{1 + from.dot(to), v.x, v.y, v.z}.unit();
    };
    const Cylinder sharp(.23, .16), rounded(.23, .16, .025);
    const Vector3 rim = sharp.coreRadius * radial + sharp.coreHalfHeight * up + sharp.radius * normal;
    const Vector3 cap = .3 * rounded.coreRadius * radial + (rounded.coreHalfHeight + rounded.radius) * up;
    const Quaternion qa = (rimFrame.orientation * Quaternion::fromRotationVector(.37 * axis) * align(normal, axis)).unit();
    const Quaternion qb = (rimFrame.orientation * Quaternion::fromRotationVector(-.52 * axis) * align(up, -axis)).unit();
    Body left = Body::cylinder(.23, .16, 1000, rimAnchor - qa.rotate(rim));
    Body right = Body::cylinder(.23, .16, 1000, rimAnchor - qb.rotate(cap), .025);
    left.orientation = qa;
    right.orientation = qb;
    World world;
    world.gravity = {};
    world.sleeping = false;
    world.realtime = true;
    world.soft.substeps = world.soft.contactUpdates = 1;
    world.addBody(left);
    world.addBody(right);
    return world;
}

void startRimAtRest(World& world, double time) {
    if (time == 0) world.sleeping = false; // Viewer::load переносит в мир общую настройку панели.
}

constexpr Real floatLift = Real(0.055);
const ExactScene gripAtLimit{
    "threshold-grip", "Пороговый захват куба, 10 Н", threshold_grip::build, {0, 0, 0}, 1.18,
    "Смещение вниз относительно губок, мм (эталон без вмешательства)",
    threshold_grip::dropMm, [](double) { return 0.0; } // 2 μ N = 10 Н, куб из покоя удерживается
};
const ExactScene water{
    "floating-box", "плавучесть куба в спокойной воде", [] { return floating_box::build(floatLift); },
    {0, 0, 0}, 0.85, "осадка куба, см",
    [](const World& world) { return 100.0 * floating_box::submergedDepth(world.body(floating_box::boxId).position.y); },
    [](double t) { return 100.0 * (floating_box::draft - floatLift * std::cos(t * std::sqrt(double(floating_box::gravity / floating_box::draft)))); }
};
const ExactScene gears{
    "gear-train", "шестерни на валах, 20 и 40 зубьев", gear_train::build, {0.10, 0, 0}, 0.47,
    "скорость выходного вала, рад/с",
    [](const World& world) { return double(world.body(gear_train::outputDisk).angularVelocity().y); },
    [](double t) { return -gear_train::ratio * double(kPi) * std::min(t, 1.0); }
};
const ExactScene rod{
    "rigid-rod", "тяга между двумя свободными телами", rigid_rod::build, {}, 1.15, "длина тяги, м",
    [](const World& w) { return (w.bodies()[1].position - w.bodies()[0].position).length(); },
    [](double) { return rigid_rod::length; }
};
const ExactScene rack{
    "rack-pinion", "реечная передача с реверсом", rack_drive::build, {.015,0,.012}, .32, "ход рейки, мм",
    [](const World& w) { return 1000 * w.body(rack_drive::rack).position.x; },
    [](double t) { const double inCycle = t - std::floor(t); return 1000 * rack_drive::radius * kPi * std::min(inCycle, 1-inCycle); }
};
const ExactScene suspension{
    "wheel-suspension", "подвеска одного колеса, 10 кг", wheel_bench::build, {0,.11,0}, .84, "высота кузова, м",
    [](const World& w) { return w.body(wheel_bench::chassis).position.y; },
    [](double t) { const double u = t * std::sqrt(wheel_bench::stiffness / wheel_bench::mass);
                   return wheel_bench::equilibrium + wheel_bench::lift * (1 + u) * std::exp(-u); }
};

} // namespace

const std::vector<Scene>& scenes() {
    static const std::vector<Scene> all = [] {
      std::vector<Scene> list = {
        {"шар сбивает стопку", ballHitsStack, {-0.5, 0.6, 0}, 5},
        {"прыгающий мяч", bouncingBall, {0, 1, 0}, 5},
        {"стопка из десяти кубов", stackOfTen, {0, 2, 0}, 8},
        {"куб падает на угол", cubeOnCorner, {0, 0.8, 0}, 5},
        {"выпуклые оболочки", fallingHulls, {0, 0.6, 0}, 7},
        {"капсулы и цилиндры", roundShapes, {0, 0.6, 0}, 7},
        {"пирамида из 55 кубов", pyramidOfFiftyFive, {0, 1.4, 0}, 9},
        {"столбик из пятидесяти кубов", columnOfFifty, {0, 2, 0}, 8},
        {"столбик из четырёхсот кубов", columnOfFourHundred, {0, 1.5, 0}, 6},
      };
      for (const ExactScene& scene : exactScenes()) list.push_back({scene.title, scene.build, scene.lookAt, scene.distance, nullptr, &scene, nullptr, "с точным ответом", scene.key});
      // Стандартный набор сцен — тот же, что считает прибор с замерами.
      for (const StandardScene& scene : standardScenes()) list.push_back({scene.title, scene.build, scene.lookAt, scene.distance, scene.control, nullptr, &scene, "стандартный набор", scene.key});
      list.push_back({"две детали: вершина, ребро и грань", hull_events::build, {.1, -.16, .32}, 2.4,
                      nullptr, nullptr, nullptr, "измеренные траектории", "hull-events"});
      list.push_back({"цилиндры: обод касается крышки", cylinderRim, rimAnchor, 1.35,
                      startRimAtRest, nullptr, nullptr, "контакт цилиндров", "cylinder-rim"});
      list.push_back({water.title, water.build, water.lookAt, water.distance, floating_box::applyBuoyancy,
                      &water, nullptr, "вода и механизмы", water.key, 8, drawWaterSurface});
      list.push_back({gears.title, gears.build, gears.lookAt, gears.distance, gear_train::drive,
                      &gears, nullptr, "вода и механизмы", gears.key, 1, drawGearMarks});
      list.push_back({gripAtLimit.title, gripAtLimit.build, gripAtLimit.lookAt, gripAtLimit.distance,
                      threshold_grip::applyLoad, &gripAtLimit, nullptr, "захват робота", gripAtLimit.key, 8, nullptr, true});
      list.push_back({"наклонное ведро на кромке", bucketsOnRim, {0, 0.33, 0}, 1.1,
                      nullptr, nullptr, nullptr, "проверка контактов", "bucket-rim"});
      list.push_back({"обрушение пирамиды из 1000 кубов", [] { return stressCubes(false); }, {0,1,0}, 9,
                      nullptr, nullptr, nullptr, "контакт и равновесие", "pyramid1000", 8, nullptr, true});
      list.push_back({"падение решётки из 1000 кубов", [] { return stressCubes(true); }, {0,2,0}, 11,
                      nullptr, nullptr, nullptr, "контакт и равновесие", "rain1000", 8, nullptr, true});
      list.push_back({"весы — равные грузы", [] { return support_scene::beam(1,1); }, {0,.13,0}, 2.6,
                      nullptr, nullptr, nullptr, "контакт и равновесие", "beam-equal", 8, nullptr, true});
      list.push_back({"весы — справа двойной груз", [] { return support_scene::beam(1,2); }, {0,.13,0}, 2.6,
                      nullptr, nullptr, nullptr, "контакт и равновесие", "beam-right", 8, nullptr, true});
      list.push_back({rod.title, rod.build, rod.lookAt, rod.distance, nullptr, &rod, nullptr, "вода и механизмы", rod.key});
      list.push_back({rack.title, rack.build, rack.lookAt, rack.distance, rack_drive::drive, &rack, nullptr, "вода и механизмы", rack.key});
      list.push_back({suspension.title, suspension.build, suspension.lookAt, suspension.distance, wheel_bench::load,
                      &suspension, nullptr, "вода и механизмы", suspension.key, 8});
      list.push_back({"машина на четырёх колёсах", vehicle_demo::build, {0, .36, 0}, 4.5,
                      nullptr, nullptr, nullptr, "вода и механизмы", "vehicle", 8, nullptr, true, &vehicle_demo::car()});
      return list;
    }();
    return all;
}

size_t sceneIndex(const char* selection) {
    const auto& list = scenes();
    const std::string_view name(selection);
    for (size_t i = 0; i < list.size(); ++i)
        if (list[i].key && name == list[i].key) return i;
    int number = 0;
    const auto parsed = std::from_chars(name.data(), name.data() + name.size(), number);
    if (parsed.ec != std::errc{} || parsed.ptr != name.data() + name.size()) return list.size();
    return size_t(std::clamp(number, 1, int(list.size())) - 1);
}
