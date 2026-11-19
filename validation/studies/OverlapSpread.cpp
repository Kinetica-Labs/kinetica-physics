// Прибор разброса перекрытия: худшее перекрытие сцены по вариантам с малыми возмущениями.
//   study-overlap-spread [имя сцены [секунд [вариантов]]]
// Сцена с ударами хаотична: начальные скорости, отличающиеся на 1 мм/с, через секунду дают другие
// удары, и худшее перекрытие с одного прогона — случайное число (у кучи из тридцати тел 6,4 мм в
// одном прогоне при медиане 12 мм и наибольшем 31 мм по шестнадцати). Поэтому сцена считается в
// нескольких вариантах — нулевой без возмущения, остальные с добавкой к скорости каждого тела до
// 1 мм/с по каждой оси, — и печатаются медиана и наибольшее. Шаг быстрый (World::realtime);
// KP_UPDATES и KP_SUBSTEPS задают SoftStep::contactUpdates и substeps.

#include <collision/PairFinder.h>
#include "../../examples/scenes/StandardScenes.h"
#include "OverlapDepth.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <typeinfo>
#include <vector>

using namespace kp;

namespace {

// SplitMix64 с постоянным зерном: варианты одни и те же от запуска к запуску. Значение в [−1, 1).
struct Random {
    std::uint64_t state;
    double roll() {
        std::uint64_t z = (state += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull, z = (z ^ (z >> 27)) * 0x94D049BB133111EBull, z ^= z >> 31;
        return double(z >> 11) / 4503599627370496.0 - 1.0;
    }
};

World variantOf(const StandardScene& scene, int variant) {
    World world = scene.build();
    world.realtime = true;
    if (const char* updates = std::getenv("KP_UPDATES")) world.soft.contactUpdates = std::atoi(updates);
    if (const char* substeps = std::getenv("KP_SUBSTEPS")) world.soft.substeps = std::atoi(substeps);
    Random random{1000 + std::uint64_t(variant)};
    for (size_t i = 0; variant > 0 && i < world.bodies().size(); ++i) {
        Body& body = world.body(BodyId(int(i)));
        if (!body.isStatic()) body.linearMomentum += (Real(1e-3) / body.mass.inverseMass) * Vector3{Real(random.roll()), Real(random.roll()), Real(random.roll())};
    }
    return world;
}

const char* kindOf(const Body& part) {
    if (part.shape->hull()) return "многогранник";
    const std::string kind = typeid(*part.shape).name();
    return kind.find("Sphere") != std::string::npos ? "шар" : kind.find("Capsule") != std::string::npos ? "капсула" : "цилиндр";
}

// Худшее перекрытие за прогон, м; who — какие формы и когда.
double worstOverlap(const StandardScene& scene, World& world, int seconds, std::string& who) {
    double worst = 0;
    for (int frame = 1; frame <= 60 * seconds; ++frame) {
        if (scene.control) scene.control(world, (frame - 1) / 60.0);
        world.step(Real(1) / 60);
        for (const ContactPair& pair : PairFinder(world.bodies(), world.gravity).pairs(0)) {
            if (!pair.inContact()) continue;
            const Body a = world.bodies()[size_t(pair.a)].part(pair.partA), b = world.bodies()[size_t(pair.b)].part(pair.partB);
            const double depth = depthOf(pair, a, b);
            if (depth <= worst) continue;
            worst = depth;
            char text[160];
            std::snprintf(text, sizeof text, "%s — %s, t = %.2f с", kindOf(a), kindOf(b), frame / 60.0);
            who = text;
        }
    }
    return worst;
}

} // namespace

int main(int argc, char** argv) {
    const std::string key = argc > 1 ? argv[1] : "pile";
    const int seconds = argc > 2 ? std::atoi(argv[2]) : 5, variants = argc > 3 ? std::atoi(argv[3]) : 16;
    for (const StandardScene& scene : standardScenes()) {
        if (key != scene.key) continue;
        std::vector<double> worsts;
        for (int variant = 0; variant < variants; ++variant) {
            World world = variantOf(scene, variant);
            std::string who;
            worsts.push_back(worstOverlap(scene, world, seconds, who));
            std::printf("  вариант %2d: %6.2f мм  (%s)\n", variant, 1e3 * worsts.back(), who.c_str());
        }
        std::sort(worsts.begin(), worsts.end());
        const double median = 0.5 * (worsts[worsts.size() / 2] + worsts[(worsts.size() - 1) / 2]);
        std::printf("%s: медиана %.2f мм, наибольшее %.2f мм, наименьшее %.2f мм по %d вариантам\n", scene.title, 1e3 * median, 1e3 * worsts.back(), 1e3 * worsts.front(), variants);
    }
    return 0;
}
