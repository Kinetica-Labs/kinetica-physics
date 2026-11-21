// Прибор «время стадий шага по кадрам»: сырьё для разбора разброса времени кадра (validation/frame_covariance.py).
// Сцена идёт кадр за кадром; наблюдатель шага (StepObserver) засекает время каждой стадии, остаток кадра
// вне стадий идёт в «прочее». Кадр — сумма стадий, поэтому дисперсия времени кадра раскладывается по ним
// точно: Var(T) = Σ Cov(s_k, T). Рядом с временем пишутся счётчики кадра, снятые после шага вне времени:
// сколько тел не спит, пар в касании, точек контакта, наибольшая скорость тела.
//   study-frame-covariance pull [exact|realtime] [сцена] [тело] [скорость] [dx dy dz]  — рывок, как в окне, 240 кадров
//   study-frame-covariance run [сцена] [секунд]                                       — стандартная сцена с начала
//   study-frame-covariance bench [сторона]                                            — пирамида замера скорости, 60 кадров
// Время каждой стадии каждого кадра — наименьшее по KP_REPEATS прогонам (3): счёт повторяется бит в бит,
// помеха время только прибавляет. KP_THREADS — потоков пула; KP_SUBSTEPS, KP_UPDATES, KP_ITERATIONS —
// настройки быстрого шага (PullScenario.h). Вывод: строка-шапка и по строке JSON на кадр.

#include <collision/PairFinder.h>
#include <math/WorkerPool.h>
#include "PullScenario.h"
#include "common.h"

#include <array>
#include <chrono>
#include <cstdio>
#include <functional>
#include <vector>

using namespace kp;

namespace {

using Clock = std::chrono::steady_clock;
constexpr size_t kStages = size_t(StepStage::Memory) + 1;
const char* const kNames[kStages + 1] = {"separation", "impact", "forces", "motion", "pairs", "contacts", "warm_start", "joints", "normals", "recheck", "friction", "memory", "other"};

double milliseconds(Clock::duration time) { return std::chrono::duration<double, std::milli>(time).count(); }

// Время с прошлой стадии идёт на счёт наступившей.
struct StageClock : StepObserver {
    std::array<double, kStages> spent{};
    Clock::time_point last;
    void after(StepStage stage, const World&) override {
        const Clock::time_point now = Clock::now();
        spent[size_t(stage)] += milliseconds(now - last), last = now;
    }
};

struct Frame {
    std::array<double, kStages + 1> ms{}; // стадии и «прочее»
    int awake = 0, pairs = 0, points = 0;
    double speed = 0;
};

// Опыт: build строит мир, prepare возвращает действие перед каждым кадром (рука, приводы).
struct Experiment {
    std::string name;
    int frames;
    std::function<World()> build;
    std::function<std::function<void(World&, int)>(World&)> prepare;
};

void count(const World& world, Frame& frame) {
    for (size_t i = 0; i < world.bodies().size(); ++i) {
        const Body& body = world.bodies()[i];
        if (body.isStatic()) continue;
        frame.awake += !world.asleep(BodyId(int(i))), frame.speed = std::max(frame.speed, double(body.linearVelocity().length()));
    }
    for (const ContactPair& pair : PairFinder(world.bodies(), world.gravity).pairs(0))
        if (pair.inContact()) ++frame.pairs, frame.points += pair.manifold.count;
}

std::vector<Frame> play(const Experiment& experiment, bool counted) {
    World world = experiment.build();
    const std::function<void(World&, int)> before = experiment.prepare(world);
    StageClock clock;
    world.observer = &clock;
    std::vector<Frame> frames(static_cast<size_t>(experiment.frames));
    for (int f = 0; f < experiment.frames; ++f) {
        before(world, f);
        clock.spent.fill(0);
        const Clock::time_point start = clock.last = Clock::now();
        world.step(Real(1) / 60);
        Frame& frame = frames[size_t(f)];
        frame.ms[kStages] = milliseconds(Clock::now() - start);
        for (size_t k = 0; k < kStages; ++k) frame.ms[k] = clock.spent[k], frame.ms[kStages] -= clock.spent[k];
        if (counted) count(world, frame);
    }
    return frames;
}

World worldOf(const SceneSpec& spec) {
    World world;
    for (const BoxSpec& b : spec.boxes) {
        Body body = Body::box({b.half.x, b.half.y, b.half.z}, b.density, {b.at.x, b.at.y, b.at.z});
        body.orientation = {b.turn.w, b.turn.x, b.turn.y, b.turn.z};
        world.addBody(body);
    }
    world.realtime = true;
    stepSettings(world);
    return world;
}

Experiment experimentOf(int argc, char** argv) {
    const std::string kind = argc > 1 ? argv[1] : "pull";
    const auto nothing = [](World&) { return [](World&, int) {}; };
    if (kind == "bench") {
        const int side = argc > 2 ? std::atoi(argv[2]) : 14;
        return {"bench " + std::to_string(side), 60, [side] { return worldOf(pyramid3d(side)); }, nothing};
    }
    if (kind == "run") {
        const std::string key = argc > 2 ? argv[2] : "wall";
        for (const StandardScene& scene : standardScenes()) {
            if (key != scene.key) continue;
            const auto build = [scene] { World world = scene.build(); world.realtime = true; stepSettings(world); return world; };
            const auto control = [scene](World&) { return [scene](World& world, int f) { if (scene.control) scene.control(world, f / 60.0); }; };
            return {"run " + key, int(60 * (argc > 3 ? std::atof(argv[3]) : scene.seconds)), build, control};
        }
    }
    const PullScenario pull = PullScenario::parse(argc, argv, 2);
    const auto grab = [pull](World& world) {
        const PullScenario::Grip grip = pull.grab(world);
        return [pull, grip](World& now, int f) { pull.drag(now, grip, f); };
    };
    return {"pull " + pull.mode + " " + pull.key + " " + std::to_string(pull.held), 240, [pull] { return pull.settled(); }, grab};
}

} // namespace

int main(int argc, char** argv) {
    if (const char* threads = std::getenv("KP_THREADS")) WorkerPool::shared().resize(std::atoi(threads));
    const Experiment experiment = experimentOf(argc, argv);
    const int repeats = std::getenv("KP_REPEATS") ? std::max(1, std::atoi(std::getenv("KP_REPEATS"))) : 3;
    std::vector<Frame> frames = play(experiment, true);
    for (int r = 1; r < repeats; ++r) {
        const std::vector<Frame> again = play(experiment, false);
        for (size_t f = 0; f < frames.size(); ++f)
            for (size_t k = 0; k <= kStages; ++k) frames[f].ms[k] = std::min(frames[f].ms[k], again[f].ms[k]);
    }
    const World sample = experiment.build();
    std::printf("{\"experiment\":\"%s\",\"threads\":%d,\"substeps\":%d,\"updates\":%d,\"iterations\":%d,\"repeats\":%d,\"stages\":[", experiment.name.c_str(),
                int(WorkerPool::shared().threads()), sample.soft.substeps, sample.soft.contactUpdates, sample.soft.directIterations, repeats);
    for (size_t k = 0; k <= kStages; ++k) std::printf("%s\"%s\"", k ? "," : "", kNames[k]);
    std::printf("]}\n");
    for (size_t f = 0; f < frames.size(); ++f) {
        std::printf("{\"f\":%zu,\"ms\":[", f);
        for (size_t k = 0; k <= kStages; ++k) std::printf("%s%.5f", k ? "," : "", std::max(0.0, frames[f].ms[k]));
        std::printf("],\"awake\":%d,\"pairs\":%d,\"points\":%d,\"speed\":%.5g}\n", frames[f].awake, frames[f].pairs, frames[f].points, frames[f].speed);
    }
}
