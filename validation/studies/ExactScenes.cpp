// Сцены с точным решением (examples/scenes/ExactScenes.h) обоими шагами: наибольшее отклонение
// величины от точного значения и наибольшее расхождение сочленений за время опыта.
//
//   study-exact-scenes [секунд, по умолчанию 10]
#include "../../examples/scenes/ExactScenes.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace kp;

int main(int argc, char** argv) {
    const double seconds = argc > 1 ? std::atof(argv[1]) : 10;
    std::printf("%-36s %-8s %-22s %12s %12s %10s %9s\n", "сцена", "шаг", "величина", "размах", "ошибка", "разрыв мм", "мс/кадр");
    for (const ExactScene& scene : exactScenes())
        for (bool realtime : {false, true}) {
            World world = scene.build();
            world.realtime = realtime;
            world.maxSegments = 64;
            double worst = 0, gap = 0, low = scene.exact(0), high = low, unfinished = 0;
            const auto start = std::chrono::steady_clock::now();
            const int frames = int(seconds * 60);
            for (int frame = 1; frame <= frames; ++frame) {
                world.step(Real(1) / 60);
                unfinished += double(world.report().unfinished);
                const double exact = scene.exact(frame / 60.0);
                worst = std::max(worst, std::fabs(scene.measured(world) - exact));
                low = std::min(low, exact), high = std::max(high, exact);
                gap = std::max(gap, double(world.joints().largestError(world.bodies())));
            }
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() / frames;
            std::printf("%-36s %-8s %-22s %12.4g %12.3e %10.4f %9.3f%s\n", scene.title, realtime ? "быстрый" : "точный", scene.quantity, high - low, worst, 1e3 * gap, ms,
                        unfinished > 0 ? "  (отстал от часов)" : "");
            std::fflush(stdout);
        }
}
