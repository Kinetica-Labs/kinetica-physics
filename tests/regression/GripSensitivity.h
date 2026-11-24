#pragma once
// Одинаковые силы, массы и шаги для трёх адаптеров. Погрешность считается относительно μ именно этой строки.
#include "GripLimit.h"

namespace grip {
inline void sensitivityHeader() {
    std::puts("engine,setting,value,hz,mu,load_N,limit_N,expected_mm,drop_mm,max_error_mm,left_N,right_N,complete");
}

template<class MakeRig>
bool sensitivity(const char* engine, const char* setting, double value, MakeRig makeRig) {
    bool complete = true;
    for (int hz : {60, 120}) for (double mu : {0.49, 0.50, 0.51}) for (double force : {9.8, 10.0, 10.2}) {
        const Load load{mu, 10, force};
        auto rig = makeRig(load);
        const auto result = measure(rig, load, hz);
        std::printf("%s,%s,%.9g,%d,%.2f,%.1f,%.6g,%.12g,%.12g,%.12g,%.12g,%.12g,%d\n",
                    engine, setting, value, hz, mu, force, 2 * mu * load.squeeze,
                    1000 * acceleration(load) * 0.25 * 0.25 / 2, 1000 * result.drop,
                    1000 * result.maxPathError, result.meanLeftN, result.meanRightN, int(result.complete));
        complete &= result.complete;
    }
    return complete;
}
}
