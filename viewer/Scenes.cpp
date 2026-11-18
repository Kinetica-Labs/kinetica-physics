#include "Scenes.h"

#include "../examples/scenes/StandardScenes.h"

using namespace kp;

namespace {

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
      for (const ExactScene& scene : exactScenes()) list.push_back({scene.title, scene.build, scene.lookAt, scene.distance, nullptr, &scene});
      // Стандартный набор сцен — тот же, что считает прибор с замерами.
      for (const StandardScene& scene : standardScenes()) list.push_back({scene.title, scene.build, scene.lookAt, scene.distance, scene.control});
      return list;
    }();
    return all;
}
