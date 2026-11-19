// Прибор главного эксперимента: стопка кубов и тяжёлый куб на лёгком на новом ядре.
// Те же сцены, что в сериях A и C табло, но с числами о работе решателя.
//   study-stack [наибольшее N стопки]

#include <collision/shapes/ConvexHull.h>
#include <world/World.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace kp;

namespace {

const double kHalf = 0.1;

Body cube(double y, double density) {
    Body body;
    body.shape = std::make_shared<ConvexHull>(ConvexHull::box({kHalf, kHalf, kHalf}));
    body.mass = MassProperties::box({kHalf, kHalf, kHalf}, density);
    body.position = {0, y, 0};
    return body;
}

struct Outcome {
    double heightError = 0, offset = 0, penetration = 0, lateSpeed = 0, energyGain = 0, seconds = 0;
    int segments = 0, iterations = 0;
};

double energy(const World& world) {
    double total = 0;
    for (const Body& b : world.bodies())
        if (!b.isStatic()) total += kineticEnergyOf(b) - world.gravity.dot(b.position) / b.mass.inverseMass;
    return total;
}

// Сцена: плита, на ней кубы с заданными высотами центров и плотностями; frames кадров по 1/60 с.
bool gShockPropagation = false; // третий аргумент программы: 1 — включить проход ударной волны

Outcome run(const std::vector<double>& heights, const std::vector<double>& densities, int frames) {
    World world;
    world.shockPropagation = gShockPropagation;
    Body ground;
    ground.shape = std::make_shared<ConvexHull>(ConvexHull::box({100, 0.5, 100}));
    ground.position = {0, -0.5, 0};
    world.addBody(ground);
    std::vector<BodyId> ids;
    for (size_t i = 0; i < heights.size(); ++i) ids.push_back(world.addBody(cube(heights[i], densities[i])));

    Outcome out;
    const auto start = std::chrono::steady_clock::now();
    double previous = energy(world);
    for (int f = 0; f < frames; ++f) {
        world.step(1.0 / 60);
        out.segments += world.report().segments;
        out.iterations = std::max(out.iterations, world.report().solverIterations);
        // Механическая энергия не имеет права расти: внешних сил, кроме тяжести, нет.
        const double now = energy(world);
        out.energyGain = std::max(out.energyGain, now - previous);
        previous = now;
        if (f >= frames - 60)
            for (BodyId id : ids) out.lateSpeed = std::max(out.lateSpeed, double(world.body(id).linearVelocity().length()));
    }
    out.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    double below = -kHalf;
    for (BodyId id : ids) {
        const Vector3 p = world.body(id).position;
        out.offset = std::max(out.offset, std::hypot(p.x, p.z));
        out.penetration = std::max(out.penetration, 2 * kHalf - (p.y - below));
        below = p.y;
    }
    out.heightError = world.body(ids.back()).position.y - (2 * double(ids.size()) - 1) * kHalf;
    return out;
}

void print(const char* label, const Outcome& o) {
    std::printf("%-16s высота %+9.3f мм  увод %8.3f мм  проникн. %8.4f мм  скорость %.1e м/с  прирост энергии %.1e Дж  "
                "отрезков %5d  проходов до %4d  %.1f с\n",
                label, 1e3 * o.heightError, 1e3 * o.offset, 1e3 * o.penetration, o.lateSpeed, o.energyGain, o.segments, o.iterations,
                o.seconds);
    std::fflush(stdout);
}

} // namespace

int main(int argc, char** argv) {
    const int largest = argc > 1 ? std::atoi(argv[1]) : 50, smallest = argc > 2 ? std::atoi(argv[2]) : 0;
    gShockPropagation = argc > 3 && std::atoi(argv[3]) != 0;
    std::printf("Стопка: кубы 0.2 м, каждый отпущен с 1 см над нижним, 10 с\n");
    for (int n : {10, 25, 50, 100, 200}) {
        if (n > largest) break;
        if (n < smallest) continue;
        std::vector<double> heights, densities;
        for (int i = 0; i < n; ++i) heights.push_back(kHalf + i * (2 * kHalf + 0.01)), densities.push_back(500);
        char label[32];
        std::snprintf(label, sizeof label, "N = %d", n);
        print(label, run(heights, densities, 600));
    }
    std::printf("Отношение масс: тяжёлый куб на лёгком, 5 с\n");
    for (double ratio : {1.0, 1e2, 1e3, 1e4, 1e6}) {
        char label[32];
        std::snprintf(label, sizeof label, "1 : %g", ratio);
        print(label, run({kHalf, 3 * kHalf}, {500, 500 * ratio}, 300));
    }
    return 0;
}
