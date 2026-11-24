#pragma once
// Один опыт для всех движков. Оси мира: x вдоль сжатия, y вверх. Единицы СИ.
#include "../../examples/scenes/GripGeometry.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace grip {

inline Box jawPosition(Box jaw, bool open) {
    if (open) jaw.x += std::copysign(1.0, jaw.x);
    return jaw;
}
struct Reading {
    double relativeY; // высота центра куба над средней высотой центров пальцев [м]
    double leftPx, rightPx; // импульсы пальцев по оси x [Н·с]
    bool valid() const {
        return std::isfinite(relativeY) && std::isfinite(leftPx) && std::isfinite(rightPx);
    }
};
struct Result {
    double drop = 0, maxTravel = 0, maxPathError = 0; // м, положительное падение — вниз
    double meanLeftN = 0, meanRightN = 0;
    double maxLeftError = 0, maxRightError = 0; // Н, худшая ошибка реакции за кадр
    int firstViolation = 0; // номер нагруженного кадра, начиная с 1
    bool complete = false;
};

// Куб и обе губки покоятся в начале. В этой симметричной задаче обе касательные вертикальны.
// При одной μ для покоя и скольжения a = sign(F) max(|F| − 2 μ N, 0) / m.
inline double acceleration(Load load) {
    const double capacity = load.open ? 0 : 2 * load.friction * load.squeeze;
    return std::copysign(std::max(0.0, std::abs(load.down) - capacity), load.down) / cubeMass;
}

template<class Rig> Result measure(Rig& rig, Load load, int hz) {
    const int frames = hz / 4; // по 0,25 с на предварительное обжатие и на измерение
    const double dt = 1.0 / hz, a = acceleration(load);
    Result r;
    for (int i = 0; i < frames; ++i)
        if (!rig.step(dt, load.squeeze, 0)) return r;
    Reading last = rig.read();
    if (!last.valid()) return r;
    const double startY = last.relativeY;
    for (int i = 1; i <= frames; ++i) {
        if (!rig.step(dt, load.squeeze, load.down)) return r;
        const Reading now = rig.read();
        if (!now.valid()) return r;
        // На свободной оси ползуна F_конт = Δp/Δt − F_прил. Знаки нормалей у пальцев разные.
        const double leftN = load.squeeze - (now.leftPx - last.leftPx) / dt;
        const double rightN = load.squeeze + (now.rightPx - last.rightPx) / dt;
        const double t = i * dt;
        r.drop = startY - now.relativeY;
        r.maxTravel = std::max(r.maxTravel, std::abs(r.drop));
        r.maxPathError = std::max(r.maxPathError, std::abs(r.drop - 0.5 * a * t * t));
        const double expectedN = load.open ? 0 : load.squeeze;
        r.meanLeftN += leftN / frames;
        r.meanRightN += rightN / frames;
        r.maxLeftError = std::max(r.maxLeftError, std::abs(leftN - expectedN));
        r.maxRightError = std::max(r.maxRightError, std::abs(rightN - expectedN));
        const bool forceOk = std::max(r.maxLeftError, r.maxRightError) <= 0.2;
        const bool pathOk = a == 0 ? r.maxTravel <= 0.0005 : r.maxPathError <= 0.002;
        if (!(forceOk && pathOk) && !r.firstViolation) r.firstViolation = i;
        last = now;
    }
    r.complete = true;
    return r;
}

// KP_GRIP_TIMING=1: время только 15 нагруженных кадров при 60 Гц. Каждый повтор начинает новый захват.
template<class MakeRig> bool timeLoadedFrames(const char* engine, MakeRig makeRig) {
    constexpr Load load{0.5, 10, 10};
    constexpr int frames = 15, warmups = 3;
    const char* profile = std::getenv("KP_GRIP_PROFILE");
    const int repeats = profile && profile[0] == '1' && profile[1] == '\0' ? 1000 : 24;
    constexpr double dt = 1.0 / 60;
    using Clock = std::chrono::steady_clock;
    double elapsedUs = 0, worstTravel = 0;
    for (int run = -warmups; run < repeats; ++run) {
        auto rig = makeRig(load);
        for (int i = 0; i < frames; ++i)
            if (!rig.step(dt, load.squeeze, 0)) return false;
        const auto initial = rig.read();
        const auto start = Clock::now();
        for (int i = 0; i < frames; ++i)
            if (!rig.step(dt, load.squeeze, load.down)) return false;
        const auto stop = Clock::now();
        const auto final = rig.read();
        if (!initial.valid() || !final.valid()) return false;
        if (run >= 0) {
            elapsedUs += std::chrono::duration<double, std::micro>(stop - start).count();
            worstTravel = std::max(worstTravel, std::abs(final.relativeY - initial.relativeY));
        }
    }
    std::printf("%s,%d,%.9g,%.9g\n", engine, repeats * frames, elapsedUs / (repeats * frames), 1000 * worstTravel);
    return true;
}
inline bool timing() {
    const char* flag = std::getenv("KP_GRIP_TIMING");
    return flag && flag[0] == '1' && flag[1] == '\0';
}
inline void header() {
    if (timing()) {
        std::puts("engine,frames,us_per_frame,end_drift_mm");
        return;
    }
    std::puts("engine,hz,mu,jaw_N,load_N,open,expected_drop_mm,drop_mm,max_travel_mm,max_path_error_mm,"
              "mean_left_N,mean_right_N,max_left_force_error_N,max_right_force_error_N,first_violation_frame,complete,pass");
}
template<class MakeRig> bool sweep(const char* engine, MakeRig makeRig) {
    if (timing()) return timeLoadedFrames(engine, makeRig);
    const Load cases[] = {{0.5,10,0}, {0.5,10,8}, {0.5,10,9.8}, {0.5,10,9.9}, {0.5,10,10},
                          {0.5,10,10.1}, {0.5,10,10.2}, {0.5,10,12}, {0.5,10,-12}, {0,10,10}, {0.5,0,10},
                          {0.5,10,10,true}};
    bool pass = true;
    for (int hz : {60, 120, 240}) {
        for (Load load : cases) {
            auto rig = makeRig(load);
            const Result r = measure(rig, load, hz);
            const bool ok = r.complete && !r.firstViolation;
            const double expected = 1000 * 0.5 * acceleration(load) * 0.25 * 0.25;
            std::printf("%s,%d,%.2f,%.1f,%.2f,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,%d\n",
                        engine, hz, load.friction, load.squeeze, load.down, int(load.open), expected,
                        1000 * r.drop, 1000 * r.maxTravel, 1000 * r.maxPathError, r.meanLeftN, r.meanRightN,
                        r.maxLeftError, r.maxRightError, r.firstViolation, int(r.complete), int(ok));
            pass &= ok;
        }
    }
    return pass;
}

} // namespace grip
