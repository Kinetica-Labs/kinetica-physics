// Два колеса на неподвижных параллельных осях. На входе постоянный момент,
// на выходе — момент нагрузки. За секунду из покоя ожидаются +90° и −45°.
#include "../scenes/GearTrain.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string_view>

using namespace kp;

namespace {
using namespace gear_train;

bool run(bool realtime, bool csv) {
    World world = build();
    world.realtime = realtime;
    const auto& input = world.joint<HingeJoint>(0);
    const auto& output = world.joint<HingeJoint>(1);
    Real phaseError = 0, speedError = 0;
    for (int frameNumber = 1; frameNumber <= 60; ++frameNumber) {
        drive(world, double(frameNumber - 1) / 60);
        world.step(Real(1) / 60);
        if (world.report().stalled || world.report().unfinished != 0) return false;
        const Real angleA = coordinate(world, input), angleB = coordinate(world, output);
        phaseError = std::max(phaseError, std::abs(angleB + ratio * angleA));
        const Real expected = kPi * frameNumber / 60;
        speedError = std::max({speedError, std::abs(world.body(inputDisk).angularVelocity().y - expected),
                               std::abs(world.body(outputDisk).angularVelocity().y + ratio * expected)});
        if (csv) {
            const Real t = Real(frameNumber) / 60;
            std::printf("%s,%d,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n", realtime ? "realtime" : "exact",
                        frameNumber, double(t), double(kPi * t * t / 2), double(angleA), double(angleB),
                        double(expected), double(world.body(inputDisk).angularVelocity().y), double(world.body(outputDisk).angularVelocity().y));
        }
    }
    const Real angle = coordinate(world, input), angleB = coordinate(world, output);
    if (!csv)
        std::printf("%s: A=%.6f deg, B=%.6f deg, max phase error %.3g rad, max speed error %.3g rad/s\n",
                    realtime ? "realtime" : "exact", double(angle * 180 / kPi), double(angleB * 180 / kPi),
                    double(phaseError), double(speedError));
    // У быстрого шага с 8 подшагами ошибка положения при постоянном α равна αTh/2.
    return std::abs(angle - kPi / 2) <= Real(0.0035) && phaseError <= Real(1e-6) && speedError <= Real(1e-7);
}
}

int main(int argc, char** argv) {
    const bool csv = argc == 2 && std::string_view(argv[1]) == "--csv";
    if (argc != 1 && !csv) return 2;
    if (csv) std::puts("mode,frame,time_s,oracle_a_rad,a_rad,b_rad,oracle_omega_a_rad_s,omega_a_rad_s,omega_b_rad_s");
    const bool exact = run(false, csv);
    const bool realtime = run(true, csv);
    return exact && realtime ? 0 : 1;
}
