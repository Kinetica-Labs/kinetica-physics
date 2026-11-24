// Одна сцена shared8 в одном потоке. stdout — фазы памяти, stderr — траектория по кадрам.
// Линковать с MemoryCounters.cpp и той же статической библиотекой, что и study-grip-limit.
// Считаются успешные global new/new[] (включая aligned). Подсчёт пропускает прямой malloc
// блоков StepMemory, внутренние выдачи арены и освобождения.
#define main kineticaGripMain
#include "../GripLimit.cpp"
#undef main

#include "../memory/MemoryCounters.h"
#include <array>

namespace {
constexpr int framesPerPhase = 15;
constexpr double frameDt = 1.0 / 60;
constexpr grip::Load load{0.5, 10, 10};

struct Phase {
    memory_study::Counts counts{};
    int frames = 0;
    bool complete = false;
};

struct Motion {
    Phase phase;
    std::array<grip::Reading, framesPerPhase> readings{};
};

Motion advanceFrames(Rig& rig, double down) {
    Motion result{};
    memory_study::start();
    for (auto& reading : result.readings) {
        if (!rig.step(frameDt, load.squeeze, down)) break;
        reading = rig.read();
        ++result.phase.frames;
        if (!reading.valid()) break;
    }
    result.phase.counts = memory_study::stop();
    result.phase.complete = result.phase.frames == framesPerPhase && result.readings.back().valid();
    return result;
}

void writePhase(const char* solver, const char* name, const Phase& phase) {
    std::printf("%s,%s,%zu,%d,%d,%zu,%zu,%d\n", solver, name, sizeof(kp::Real),
                phase.frames, 8 * phase.frames, phase.counts.heapCalls, phase.counts.heapBytes,
                int(phase.complete));
}

void writeMotion(const char* solver, const char* name, const Motion& motion, int elapsedFrames) {
    for (int i = 0; i < motion.phase.frames; ++i) {
        const auto& reading = motion.readings[i];
        std::fprintf(stderr, "%s,%s,%d,%.17g,%.17g,%.17g,%.17g\n", solver, name, i + 1,
                     (elapsedFrames + i + 1) * frameDt, reading.relativeY, reading.leftPx, reading.rightPx);
    }
}
}

int main(int argc, char** argv) {
    const std::string_view mode = argc == 2 ? argv[1] : "";
    if (argc > 2 || (argc == 2 && mode != "stock" && mode != "coupled")) {
        std::fputs("kinetica-memory [stock|coupled]\n", stderr);
        return 2;
    }
    const bool coupled = mode == "coupled";
    const char* solver = coupled ? "coupled" : "stock";
    kp::WorkerPool::shared().resize(1); // запуск инфраструктуры предшествует созданию мира
    Phase construction{}, destruction{};
    Motion preload{}, loaded{};
    memory_study::start();
    {
        Rig rig(1, load, true);
        rig.world().soft.coupledFriction = coupled;
        construction.counts = memory_study::stop();
        construction.complete = construction.counts.heapCalls > 0 && rig.read().valid();
        if (construction.complete) preload = advanceFrames(rig, 0);
        if (preload.phase.complete) loaded = advanceFrames(rig, load.down);
        memory_study::start();
    }
    destruction.counts = memory_study::stop();
    destruction.complete = true;

    std::puts("solver,phase,scalar_bytes,completed_frames,public_steps_in_completed_frames,"
              "global_new_calls,global_new_requested_B,complete");
    writePhase(solver, "construction", construction);
    writePhase(solver, "preload", preload.phase);
    writePhase(solver, "loaded", loaded.phase);
    writePhase(solver, "destruction", destruction);
    std::fputs("solver,phase,frame_in_phase,t_world_s,relative_y_m,left_px_Ns,right_px_Ns\n", stderr);
    writeMotion(solver, "preload", preload, 0);
    writeMotion(solver, "loaded", loaded, preload.phase.frames);
    return loaded.phase.complete ? 0 : 1;
}
