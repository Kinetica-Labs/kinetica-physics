// Прибор этапов: на что уходит кадр быстрого шага.
//   study-frame-stages [имя сцены [секунд]]        по умолчанию все стандартные сцены, время сцены
//   study-frame-stages pyramid3d [секунд]          трёхмерная пирамида замера скорости — та же, что у
//                                                  BENCH=14 compare-ours; KP_SIDE — сторона основания (14)
// Время кадра делится по стадиям шага (dynamics/StepStage.h): наблюдатель мира получает управление
// после каждой стадии, и время с прошлого вызова идёт на её счёт. Печатает миллисекунды на кадр по
// стадиям, а для одной названной сцены — ещё и доли кадра. «пары» — поиск пар вместе с точками
// контакта; «прочее» — сон островов и всё, что вне стадий. KP_THREADS — число потоков пула.

#include "common.h"
#include <math/WorkerPool.h>
#include "../../examples/scenes/StandardScenes.h"

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>
#include <vector>

using namespace kp;

namespace {

using Clock = std::chrono::steady_clock;
constexpr StepStage kStages[] = {StepStage::Pairs,   StepStage::Contacts, StepStage::WarmStart, StepStage::Joints, StepStage::Normals,
                                 StepStage::Recheck, StepStage::Motion,   StepStage::Friction,  StepStage::Memory};

double milliseconds(Clock::duration time) { return std::chrono::duration<double, std::milli>(time).count(); }

// Время с прошлой стадии идёт на счёт наступившей.
struct StageClock : StepObserver {
    std::array<double, size_t(StepStage::Memory) + 1> spent{}; // мс по номеру стадии
    Clock::time_point last;
    void after(StepStage stage, const World&) override {
        const Clock::time_point now = Clock::now();
        spent[size_t(stage)] += milliseconds(now - last), last = now;
    }
};

struct Scene {
    std::string key;
    std::function<World()> build;
    double seconds;
    void (*control)(World& world, double time);
};

World worldOf(const SceneSpec& spec) {
    World world;
    for (const BoxSpec& b : spec.boxes) {
        Body body = Body::box({b.half.x, b.half.y, b.half.z}, b.density, {b.at.x, b.at.y, b.at.z});
        body.orientation = {b.turn.w, b.turn.x, b.turn.y, b.turn.z};
        world.addBody(body);
    }
    return world;
}

std::vector<Scene> measured() {
    std::vector<Scene> all;
    for (const StandardScene& scene : standardScenes()) all.push_back({scene.key, scene.build, scene.seconds, scene.control});
    const char* side = std::getenv("KP_SIDE");
    all.push_back({"pyramid3d", [side] { return worldOf(pyramid3d(side ? std::atoi(side) : 14)); }, 1, nullptr});
    return all;
}

// Строка сцены: кадр и стадии, мс на кадр; shares — второй строкой доли кадра, %.
void print(const std::string& key, const StageClock& clock, double total, int frames, bool shares) {
    double staged = 0;
    for (StepStage stage : kStages) staged += clock.spent[size_t(stage)];
    std::printf("%-10s %8.2f", key.c_str(), total / frames);
    for (StepStage stage : kStages) std::printf(" %8.2f", clock.spent[size_t(stage)] / frames);
    std::printf(" %8.2f\n", (total - staged) / frames);
    if (!shares) return;
    std::printf("%-10s %8s", "", "%");
    for (StepStage stage : kStages) std::printf(" %8.1f", 100 * clock.spent[size_t(stage)] / total);
    std::printf(" %8.1f\n", 100 * (total - staged) / total);
}

} // namespace

int main(int argc, char** argv) {
    if (const char* threads = std::getenv("KP_THREADS")) WorkerPool::shared().resize(std::atoi(threads));
    const std::string only = argc > 1 ? argv[1] : "";
    std::puts("сцена      кадр, мс     пары контакты т. старт  сочлен.  нормали    доиск    сдвиг   трение   память   прочее");
    for (const Scene& scene : measured()) {
        if (!only.empty() && only != scene.key) continue;
        World world = scene.build();
        StageClock clock;
        world.realtime = true, world.observer = &clock;
        const int frames = int(60 * (argc > 2 ? std::atof(argv[2]) : scene.seconds));
        double total = 0;
        for (int frame = 0; frame < frames; ++frame) {
            if (scene.control) scene.control(world, frame / 60.0);
            const Clock::time_point start = clock.last = Clock::now();
            world.step(Real(1) / 60);
            total += milliseconds(Clock::now() - start);
        }
        print(scene.key, clock, total, frames, !only.empty());
        std::fflush(stdout);
    }
    return 0;
}
