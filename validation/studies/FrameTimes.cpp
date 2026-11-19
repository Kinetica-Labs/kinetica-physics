// Прибор рывков: распределение времени кадра быстрого шага.
//   study-frame-times [имя сцены [секунд]]        по умолчанию все стандартные сцены, время сцены
// Среднее время кадра рывков не показывает: глаз видит отдельный кадр, который не уложился в
// 16,7 мс. Печатает медиану, 99-й процентиль и наибольшее время кадра, а также число кадров
// длиннее 16,7 мс и длиннее пяти медиан. KP_THREADS — число потоков пула; KP_SLEEP=1 — со сном;
// KP_PACED=1 — кадр на 1/60 с, как в окне: между шагами процесс спит, ядра успевают сбросить частоту,
// и видна цена их разгона; KP_PACED=2 — тот же темп, но ожидание без сна (ядро остаётся разогнанным).

#include <math/WorkerPool.h>
#include "../../examples/scenes/StandardScenes.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

using namespace kp;

int main(int argc, char** argv) {
    if (const char* threads = std::getenv("KP_THREADS")) WorkerPool::shared().resize(std::atoi(threads));
    const std::string only = argc > 1 ? argv[1] : "";
    std::printf("%-10s %7s %9s %9s %9s %10s %12s\n", "сцена", "кадров", "медиана", "99 %", "наиб.", "> 16,7 мс", "> 5 медиан");
    for (const StandardScene& scene : standardScenes()) {
        if (!only.empty() && only != scene.key) continue;
        World world = scene.build();
        world.realtime = true, world.sleeping = std::getenv("KP_SLEEP") != nullptr;
        const int frames = int(60 * (argc > 2 ? std::atof(argv[2]) : scene.seconds));
        std::vector<double> times;
        for (int frame = 0; frame < frames; ++frame) {
            if (scene.control) scene.control(world, frame / 60.0);
            const auto start = std::chrono::steady_clock::now();
            world.step(Real(1) / 60);
            times.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
            const char* paced = std::getenv("KP_PACED");
            if (paced && paced[0] == '1') std::this_thread::sleep_until(start + std::chrono::microseconds(16667));
            while (paced && paced[0] == '2' && std::chrono::steady_clock::now() < start + std::chrono::microseconds(16667)) {} // ждём, не засыпая
        }
        std::sort(times.begin(), times.end());
        const double median = times[times.size() / 2];
        const auto longer = [&](double limit) { return int(times.end() - std::upper_bound(times.begin(), times.end(), limit)); };
        std::printf("%-10s %7d %9.2f %9.2f %9.2f %10d %12d\n", scene.key, frames, median, times[times.size() * 99 / 100], times.back(), longer(16.7), longer(5 * median));
        std::fflush(stdout);
    }
    return 0;
}
