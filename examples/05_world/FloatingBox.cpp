// Вертикальный брусок в спокойной воде. Поверхность на y = 0, сопротивление среды равно нулю.
#include "../scenes/FloatingBox.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {
using namespace kp;
using namespace floating_box;

struct Motion { Real maxOffset = 0, period = 0; };

Motion observe(Real lift) {
    World world = build(lift);
    Motion motion;
    Real previous = lift, lastCrossing = -1;
    for (int i = 0; i < 960; ++i) {
        applyBuoyancy(world, i * double(dt));
        world.step(dt);
        const Real offset = world.body(boxId).position.y - equilibriumY;
        motion.maxOffset = std::max(motion.maxOffset, std::abs(offset));
        if (lift > 0 && previous > 0 && offset <= 0) {
            const Real crossing = (i + previous / (previous - offset)) * dt;
            if (lastCrossing >= 0) { motion.period = crossing - lastCrossing; break; }
            lastCrossing = crossing;
        }
        previous = offset;
    }
    return motion;
}
}

int main() {
    const Real dry = buoyantForce(submergedDepth(edge));
    const Real half = buoyantForce(submergedDepth(equilibriumY));
    const Real full = buoyantForce(submergedDepth(-edge));
    const Real weight = boxDensity * edge * edge * edge * gravity;
    const Real restDrift = observe(0).maxOffset;
    const Real period = observe(Real(0.01)).period;
    const Real expectedPeriod = 2 * kPi * kp::sqrt(draft / gravity); // m y'' = −ρ g A y
    std::printf("dry / half / full [N]: %.6f / %.6f / %.6f\n", double(dry), double(half), double(full));
    std::printf("draft [m]: %.6f, rest drift [m]: %.3g\n", double(draft), double(restDrift));
    std::printf("period [s]: %.8f, oracle: %.8f\n", double(period), double(expectedPeriod));
    const bool forcesOk = dry == 0 && std::abs(half - weight) < Real(1e-6) && std::abs(full - 2 * weight) < Real(1e-6);
    return forcesOk && restDrift < Real(1e-6) && std::abs(period - expectedPeriod) < Real(0.01) * expectedPeriod ? 0 : 1;
}
