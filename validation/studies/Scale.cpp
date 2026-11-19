// Прибор масштаба: цена кадра быстрого шага от числа тел.
//   study-scale [тел [тел ...]]        по умолчанию 1000 и 10000
// Три расклада кубов с ребром 20 см на полу:
//   порознь  — кубы лежат на полу и друг друга не касаются: островок — одно тело, лучший случай;
//   столбики — кубы стоят столбиками по четыре: островок — четыре тела;
//   кладка   — кубы уложены вплотную в четыре слоя: один островок на всю сцену.
// Печатает миллисекунды на кадр (среднее по 20 кадрам после 10 разгонных) и микросекунды на тело.
// KP_THREADS — число потоков пула; KP_SLEEP=1 — со сном островов; KP_LAYOUT=имя — один расклад.

#include <math/WorkerPool.h>
#include <physics.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace kp;

namespace {

constexpr Real kHalf = Real(0.1); // полуребро куба, м

// Куб номер k расклада: columns — сколько кубов друг на друге, gap — зазор между соседними по полу.
World layout(int count, int columns, Real gap) {
    World world;
    world.realtime = true, world.sleeping = std::getenv("KP_SLEEP") != nullptr;
    const int side = int(std::ceil(std::sqrt(double(count) / columns)));
    const Real pitch = 2 * kHalf + gap, half = pitch * side / 2 + 1;
    world.addBody(Body::box({half, Real(0.5), half}, 0, {0, Real(-0.5), 0}));
    for (int k = 0; k < count; ++k) {
        const int cell = k / columns, level = k % columns;
        const Real x = pitch * (cell % side) - pitch * side / 2, z = pitch * (cell / side) - pitch * side / 2;
        world.addBody(Body::box({kHalf, kHalf, kHalf}, 1000, {x, kHalf + 2 * kHalf * level, z}));
    }
    return world;
}

double millisecondsPerFrame(World& world) {
    for (int frame = 0; frame < 10; ++frame) world.step(Real(1) / 60);
    const auto start = std::chrono::steady_clock::now();
    for (int frame = 0; frame < 20; ++frame) world.step(Real(1) / 60);
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() / 20;
}

} // namespace

int main(int argc, char** argv) {
    if (const char* threads = std::getenv("KP_THREADS")) WorkerPool::shared().resize(std::atoi(threads));
    std::vector<int> counts;
    for (int k = 1; k < argc; ++k) counts.push_back(std::atoi(argv[k]));
    if (counts.empty()) counts = {1000, 10000};
    struct Layout {
        const char* name;
        int columns;
        Real gap;
    };
    const Layout layouts[] = {{"порознь", 1, Real(0.3)}, {"столбики", 4, Real(0.3)}, {"кладка", 4, 0}};
    std::printf("%-10s %9s %12s %12s\n", "расклад", "тел", "мс/кадр", "мкс/тело");
    const char* only = std::getenv("KP_LAYOUT"); // один расклад по имени
    for (const Layout& each : layouts)
        for (int count : counts) {
            if (only && std::string(only) != each.name) continue;
            World world = layout(count, each.columns, each.gap);
            const double cost = millisecondsPerFrame(world);
            std::printf("%-10s %9d %12.2f %12.2f\n", each.name, count, cost, 1e3 * cost / count);
            std::fflush(stdout);
        }
    return 0;
}
