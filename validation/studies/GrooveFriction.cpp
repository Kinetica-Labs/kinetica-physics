// Шар в V-образном жёлобе. После одного подшага проверяем продольное скольжение по обеим опорам.
#include "StudyFriction.h"
#include <math/WorkerPool.h>
#include <world/World.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string_view>

namespace {
constexpr double radius = 0.1, gravity = 10, step = 1.0 / 480, tolerance = 1e-6;
using Pair = std::array<double, 2>;

// Импульсы на единицу массы, м/с. Для сплошного шара I=2mR²/5.
// Минимум квадратичной энергии на прямоугольнике: внутри или на одной из четырёх сторон.
Pair contactSpeeds(double angle, double initial, Pair mu) {
    constexpr double diagonal = 3.5;
    const double cross = 1 + 2.5 * std::cos(2 * angle), speed = std::abs(initial);
    const double normal = gravity * step / (2 * std::cos(angle));
    const Pair limit{mu[0] * normal, mu[1] * normal};
    const double free = speed / (diagonal + cross);
    Pair best{free, free};
    if (free > limit[0] || free > limit[1]) {
        double least = std::numeric_limits<double>::infinity();
        for (int i : {0, 1}) for (int side : {-1, 1}) {
            Pair j{};
            j[i] = side * limit[i];
            j[1-i] = std::clamp((speed - cross * j[i]) / diagonal, -limit[1-i], limit[1-i]);
            const double energy = 0.5 * diagonal * (j[0] * j[0] + j[1] * j[1]) + cross * j[0] * j[1] - speed * (j[0] + j[1]);
            if (energy < least) { least = energy; best = j; }
        }
    }
    const double sign = std::copysign(1.0, initial);
    return {initial - sign * (diagonal * best[0] + cross * best[1]),
            initial - sign * (diagonal * best[1] + cross * best[0])};
}

void addSupport(kp::World& world, double angle, double mu) {
    constexpr kp::Real thickness = kp::Real(0.05);
    const kp::Vector3 normal{0, kp::Real(std::cos(angle)), kp::Real(std::sin(angle))};
    auto support = kp::Body::box({1, thickness, 1}, 0, -(kp::Real(radius) + thickness) * normal);
    support.orientation = kp::Quaternion::fromRotationVector({kp::Real(angle), 0, 0});
    support.friction = kp::Real(mu * mu); // у шара μ = 1, пара использует геометрическое среднее
    world.addBody(support);
}

bool measure(int degrees, double speed, Pair mu) {
    const double angle = degrees * std::acos(-1.0) / 180;
    kp::World world;
    world.soft.coupledFriction = study::coupledFrictionRequested();
    world.realtime = true;
    world.gravity = {0, kp::Real(-gravity), 0};
    world.soft.substeps = world.soft.contactUpdates = 1;
    addSupport(world, angle, mu[0]);
    addSupport(world, -angle, mu[1]);
    auto sphere = kp::Body::sphere(kp::Real(radius), 1000);
    sphere.friction = 1;
    const auto id = world.addBody(sphere);
    for (int i = 0; i < 120; ++i) {
        world.step(kp::Real(step));
        if (world.report().stalled || world.report().unfinished != 0) return false;
    }
    auto& ball = world.body(id);
    const double rest = ball.linearVelocity().length() + radius * ball.angularVelocity().length();
    ball.linearMomentum.x = kp::Real(speed) / ball.mass.inverseMass;
    world.step(kp::Real(step));
    if (world.report().stalled || world.report().unfinished != 0) return false;
    const auto w = ball.angularVelocity();
    const double mean = ball.linearVelocity().x + radius * std::cos(angle) * w.z;
    const double difference = radius * std::sin(angle) * w.y;
    const Pair observed{mean - difference, mean + difference}, expected = contactSpeeds(angle, speed, mu);
    const double error = std::max(std::abs(observed[0] - expected[0]), std::abs(observed[1] - expected[1]));
    const bool pass = rest <= tolerance && error <= tolerance;
    std::printf("%d,%.3g,%.3g,%.3g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%d\n", degrees, speed, mu[0], mu[1],
                rest, expected[0], expected[1], observed[0], observed[1], error, int(pass));
    return pass;
}
} // namespace

int main(int argc, char** argv) {
    const bool holdout = argc == 2 && std::string_view(argv[1]) == "--holdout";
    if (argc != 1 && !holdout) return 2;
    if (const char* threads = std::getenv("KP_THREADS")) kp::WorkerPool::shared().resize(std::atoi(threads));
    std::puts("angle_deg,initial_m_s,mu_left,mu_right,rest_m_s,oracle_left_m_s,oracle_right_m_s,left_m_s,right_m_s,error_m_s,pass");
    bool pass = true;
    const auto angles = holdout ? std::array<int,2>{20,55} : std::array<int,2>{30,45};
    const auto coatings = holdout ? std::array<Pair,4>{{{0.35,0.35}, {0.12,0.8}, {0.8,0.12}, {0.18,0.28}}}
                                  : std::array<Pair,4>{{{0.5,0.5}, {0.05,0.6}, {0.6,0.05}, {0,0}}};
    const auto speeds = holdout ? std::array<double,4>{-0.024,-0.002,0.024,0.065} : std::array<double,4>{-0.01,0,0.01,0.1};
    for (int angle : angles)
        for (Pair mu : coatings)
            for (double speed : speeds) pass &= measure(angle, speed, mu);
    return pass ? 0 : 1;
}
