// Прибор «рывок куба, как в окне»: стандартная сцена с настройками окна (сон тел, сопротивление среды
// 0,1 1/с, привязь мыши 8 Гц) три секунды стоит, затем тело тянут рукой полсекунды со скоростью speed по
// направлению (dx, dy, dz) и держат; всего 240 кадров. Мерка — хвост времени кадра, а не среднее: окно
// подвисает на отдельных кадрах при медиане 1–3 мс (жалоба из окна 2026-11-20, H87 и H88 в журнале
// попыток). Печатаются медиана, 99-й процентиль и наибольший кадр, счёт кадров дольше кадра экрана
// (16,7 мс) и признаки здравости: наибольшая скорость тела и худшее перекрытие (взорвавшаяся сцена
// считается быстро, и без них её время выглядит победой).
//   study-window-pull [exact|realtime] [сцена] [тело] [скорость, м/с] [dx dy dz]
// Время каждого кадра — наименьшее по KP_REPEATS прогонам (по умолчанию 3): счёт повторяется бит в бит,
// а помеха соседних программ время только прибавляет. KP_THREADS — число потоков пула; KP_SEGMENTS —
// предел отрезков точного шага (в окне 8); KP_SUBSTEPS, KP_UPDATES, KP_ITERATIONS — настройки быстрого шага
// (PullScenario.h). Последняя строка — те же числа в JSON для validation/gate.py.

#include <collision/PairFinder.h>
#include <math/WorkerPool.h>
#include "OverlapDepth.h"
#include "PullScenario.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace kp;

namespace {

struct Sanity {
    double peakSpeed = 0, overlap = 0; // м/с и м за все кадры рывка
};

// Время каждого из 240 кадров рывка, мс; sanity — если задан, меряется после каждого кадра вне времени.
std::vector<double> pullOnce(const PullScenario& pull, Sanity* sanity) {
    World world = pull.settled();
    const PullScenario::Grip grip = pull.grab(world);
    std::vector<double> times;
    for (int f = 0; f < 240; ++f) {
        pull.drag(world, grip, f);
        const auto t0 = std::chrono::steady_clock::now();
        world.step(Real(1) / 60);
        times.push_back(1e3 * std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
        if (!sanity) continue;
        for (const Body& body : world.bodies())
            if (!body.isStatic()) sanity->peakSpeed = std::max(sanity->peakSpeed, double(body.linearVelocity().length()));
        for (const ContactPair& pair : PairFinder(world.bodies(), world.gravity).pairs(0))
            if (pair.inContact()) sanity->overlap = std::max(sanity->overlap, depthOf(pair, world.bodies()[size_t(pair.a)].part(pair.partA), world.bodies()[size_t(pair.b)].part(pair.partB)));
    }
    return times;
}

} // namespace

int main(int argc, char** argv) {
    if (const char* threads = std::getenv("KP_THREADS")) WorkerPool::shared().resize(std::atoi(threads));
    const PullScenario pull = PullScenario::parse(argc, argv, 1);
    const int repeats = std::getenv("KP_REPEATS") ? std::max(1, std::atoi(std::getenv("KP_REPEATS"))) : 3;
    Sanity sanity;
    std::vector<double> times = pullOnce(pull, &sanity);
    for (int r = 1; r < repeats; ++r) {
        const std::vector<double> again = pullOnce(pull, nullptr);
        for (size_t f = 0; f < times.size(); ++f) times[f] = std::min(times[f], again[f]);
    }
    std::vector<double> sorted = times;
    std::sort(sorted.begin(), sorted.end());
    const double median = sorted[sorted.size() / 2], tail = sorted[sorted.size() * 99 / 100], worst = sorted.back();
    const int slow = int(std::count_if(times.begin(), times.end(), [](double t) { return t > 16.7; }));
    const size_t worstFrame = size_t(std::max_element(times.begin(), times.end()) - times.begin());
    std::printf("%s %s, тело %d, рука %.1f м/с: медиана %.2f мс, 99 %% %.2f, наибольший %.2f (кадр %zu), дольше 16,7 мс — %d из %zu; скорость тел до %.2f м/с, перекрытие до %.3f мм\n",
                pull.mode.c_str(), pull.key.c_str(), pull.held, pull.speed, median, tail, worst, worstFrame, slow, times.size(), sanity.peakSpeed, 1e3 * sanity.overlap);
    std::printf("{\"mode\":\"%s\",\"scene\":\"%s\",\"body\":%d,\"median_ms\":%.4f,\"p99_ms\":%.4f,\"worst_ms\":%.4f,\"slow_frames\":%d,\"peak_speed\":%.6g,\"overlap_m\":%.6g}\n",
                pull.mode.c_str(), pull.key.c_str(), pull.held, median, tail, worst, slow, sanity.peakSpeed, sanity.overlap);
}
