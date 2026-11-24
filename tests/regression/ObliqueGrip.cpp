// Скользящие полюса шара. KP_THREADS задаёт число потоков (по умолчанию 1).
#include "StudyFriction.h"
#include "ObliqueGrip.h"
#include <world/World.h>
#include <math/WorkerPool.h>
#include <cstdlib>

namespace {
using namespace oblique;

class Clamp {
public:
    explicit Clamp(std::array<double, 2> mu) {
        world.soft.coupledFriction = study::coupledFrictionRequested();
        world.realtime = true;
        world.soft.substeps = world.soft.contactUpdates = 1;
        world.gravity = {};
        auto base = world.addBody(kp::Body{});
        for (int i : {0, 1}) {
            const kp::Vector3 pos{kp::Real(jawPosition(i)), 0, 0};
            auto jaw = kp::Body::box({kp::Real(jawHalfX), kp::Real(jawHalfYZ), kp::Real(jawHalfYZ)}, jawDensity, pos);
            jaw.friction = kp::Real(mu[i] * mu[i]); // геометрическое среднее с коэффициентом шара 1
            jaws[i] = world.addBody(jaw);
            kp::addSlider(world, base, jaws[i], pos, {1,0,0});
        }
        auto ball = kp::Body::sphere(radius, sphereDensity);
        ball.friction = 1;
        sphere = world.addBody(ball);
    }
    bool step() {
        world.body(jaws[0]).force.x = force;
        world.body(jaws[1]).force.x = -force;
        world.step(h);
        return !world.report().stalled && world.report().unfinished == 0;
    }
    void launch(Pair u) {
        auto& ball = world.body(sphere);
        const auto motion = initialMotion(u);
        ball.linearMomentum = (1 / ball.mass.inverseMass) * kp::Vector3{0, kp::Real(motion.centre.real()), kp::Real(motion.centre.imag())};
        ball.angularMomentum = (1 / ball.mass.inverseInertia.x) * kp::Vector3{0, kp::Real(motion.angular.real()), kp::Real(motion.angular.imag())};
    }
    Pair poleVelocities() const {
        const auto& b = world.body(sphere);
        const auto v = b.linearVelocity(), w = b.angularVelocity();
        return poleSpeeds({v.y, v.z}, {w.y, w.z});
    }
    Pair jawVelocities() const {
        Pair result;
        for (int i : {0, 1}) {
            auto pole = world.body(sphere).position;
            pole.x += kp::Real(i == 0 ? -radius : radius);
            const auto v = kp::pointVelocity(world.body(jaws[i]), pole);
            result[i] = {v.y, v.z};
        }
        return result;
    }
    std::array<double, 2> jawMomentaX() const {
        return {world.body(jaws[0]).linearMomentum.x, world.body(jaws[1]).linearMomentum.x};
    }
private:
    kp::World world;
    std::array<kp::BodyId,2> jaws;
    kp::BodyId sphere;
};

}

int main(int argc, char** argv) {
    const char* threads = std::getenv("KP_THREADS");
    kp::WorkerPool::shared().resize(threads ? std::atoi(threads) : 1);
    return oblique::run<Clamp>(argc, argv);
}
