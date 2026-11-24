#include "Check.h"
#include "../examples/scenes/WheelBench.h"
#include <algorithm>
#include <cmath>

namespace {
void wheelLoads() {
    const kp::WheelContact wheel({10000,100,500,300,.8,.4});
    const auto unloaded = wheel.forces(-.01, 2, 1, -2);
    const auto unloading = wheel.forces(.001, -1, 1, -2);
    CHECK_NEAR(unloaded.normal + std::abs(unloaded.longitudinal) + std::abs(unloaded.lateral), 0, 0);
    CHECK_NEAR(unloading.normal + std::abs(unloading.longitudinal) + std::abs(unloading.lateral), 0, 0);
    const auto straight = wheel.forces(.01, 0, .04, 0);
    CHECK_NEAR(straight.normal, 100, 1e-12);
    CHECK_NEAR(straight.longitudinal, -20, 1e-12);
    for (double u : {-2.,0.,2.}) for (double v : {-2.,0.,2.}) {
        const auto f = wheel.forces(.01, 0, u, v);
        const double ellipse = std::pow(f.longitudinal/80, 2) + std::pow(f.lateral/40, 2);
        CHECK_NEAR(ellipse, (u == 0 && v == 0 ? 0. : 1.), 1e-12);
        CHECK(f.longitudinal*u + f.lateral*v <= 0, "positive tire friction power");
    }
    const kp::WheelContact freeRolling({10000,100,500,300,0,.4});
    const auto ice = freeRolling.forces(.01, 0, 2, 2);
    CHECK_NEAR(ice.longitudinal, 0, 0);
    CHECK_NEAR(ice.lateral, -40, 1e-12);
}

void verticalSuspension(bool realtime) {
    using namespace wheel_bench;
    kp::World w = build();
    w.realtime = realtime;
    constexpr int hz = 480;
    const double rate = std::sqrt(stiffness / mass);
    double error = 0;
    for (int i = 1; i <= hz; ++i) {
        load(w, double(i-1) / hz);
        w.step(kp::Real(1) / hz);
        CHECK(!w.report().stalled && w.report().unfinished == 0, "suspension unfinished at %d", i);
        const double u = rate * i / hz;
        const double reference = equilibrium + lift * (1 + u) * std::exp(-u);
        error = std::max(error, std::abs(double(w.body(chassis).position.y) - reference));
    }
    const auto n = tire().forces(restHeight - equilibrium, 0, 0, 0).normal;
    std::printf("wheel,fast=%d,static_load_N=%.12g,max_height_error_m=%.12g\n", int(realtime), n, error);
    CHECK_NEAR(n, mass * gravity, 1e-10);
    CHECK_NEAR(error, 0, 5e-4);
}
}
void wheelTests() {
    wheelLoads();
    verticalSuspension(false);
    verticalSuspension(true);
}
