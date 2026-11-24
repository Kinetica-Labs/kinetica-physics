// Bullet 3, btDiscreteDynamicsWorld + однопоточный sequential impulse solver.
// Для сборки: заголовки bullet3/src, затем BulletDynamics, BulletCollision, LinearMath.
// BT_USE_DOUBLE_PRECISION у адаптера и всех трёх библиотек должен совпадать.
#include "../studies/GripLimit.h"
#include <btBulletDynamicsCommon.h>
#include <charconv>
#include <string>
#include <string_view>

namespace {
// Размеры задают внешние полуширины. btBoxShape учитывает собственный margin внутри них.
struct BoxBody {
    btBoxShape shape;
    btRigidBody body;

    BoxBody(grip::Box box, double mu)
        : shape(btVector3(btScalar(box.halfX), btScalar(box.halfY), btScalar(box.halfZ))),
          body(properties(box, shape, mu)) {
        body.setActivationState(DISABLE_DEACTIVATION);
    }

    static btRigidBody::btRigidBodyConstructionInfo properties(grip::Box box, btBoxShape& shape, double mu) {
        const btScalar mass = btScalar(8 * box.halfX * box.halfY * box.halfZ * box.density);
        btVector3 inertia;
        shape.calculateLocalInertia(mass, inertia);
        btRigidBody::btRigidBodyConstructionInfo info(mass, nullptr, &shape, inertia);
        info.m_startWorldTransform.setIdentity();
        info.m_startWorldTransform.setOrigin(btVector3(btScalar(box.x), 0, 0));
        info.m_friction = btScalar(std::sqrt(mu)); // Bullet перемножает коэффициенты поверхностей.
        info.m_restitution = info.m_linearDamping = info.m_angularDamping = 0;
        return info;
    }
};

class Rig {
public:
    explicit Rig(grip::Load load, int iterations = 1)
        : dispatcher_(&configuration_), world_(&dispatcher_, &broadphase_, &solver_, &configuration_),
          left_(grip::jawPosition(grip::leftJaw, load.open), load.friction),
          right_(grip::jawPosition(grip::rightJaw, load.open), load.friction), cube_(grip::cube, load.friction),
          leftGuide_(left_.body, btTransform::getIdentity(), true),
          rightGuide_(right_.body, btTransform::getIdentity(), true) {
        world_.setGravity(btVector3(0, 0, 0));
        auto& solver = world_.getSolverInfo();
        solver.m_numIterations = iterations;
        solver.m_solverMode |= SOLVER_USE_WARMSTARTING; // множитель предыдущего импульса в этой ревизии 0,85.
        for (auto* box : {&left_, &right_, &cube_}) world_.addRigidBody(&box->body);
        for (auto* guide : {&leftGuide_, &rightGuide_}) {
            guide->setLowerLinLimit(1); // нижний > верхнего оставляет ось x свободной.
            guide->setUpperLinLimit(-1);
            guide->setLowerAngLimit(0);
            guide->setUpperAngLimit(0);
            world_.addConstraint(guide, true);
        }
    }

    ~Rig() {
        for (auto* guide : {&rightGuide_, &leftGuide_}) world_.removeConstraint(guide);
        for (auto* box : {&cube_, &right_, &left_}) world_.removeRigidBody(&box->body);
    }
    Rig(const Rig&) = delete;

    bool step(double dt, double squeeze, double down) {
        const btScalar h = btScalar(dt / 8);
        for (int i = 0; i < 8; ++i) {
            left_.body.applyCentralForce(btVector3(btScalar(squeeze), 0, 0));
            right_.body.applyCentralForce(btVector3(btScalar(-squeeze), 0, 0));
            cube_.body.applyCentralForce(btVector3(0, btScalar(-down), 0));
            if (world_.stepSimulation(h, 1, h) != 1) return false;
        }
        return true;
    }

    grip::Reading read() const {
        const double jawY = (double(left_.body.getCenterOfMassPosition().y()) + right_.body.getCenterOfMassPosition().y()) / 2;
        return {cube_.body.getCenterOfMassPosition().y() - jawY,
                double(left_.body.getMass()) * left_.body.getLinearVelocity().x(),
                double(right_.body.getMass()) * right_.body.getLinearVelocity().x()};
    }
private:
    btDefaultCollisionConfiguration configuration_;
    btCollisionDispatcher dispatcher_;
    btDbvtBroadphase broadphase_;
    btSequentialImpulseConstraintSolver solver_;
    btDiscreteDynamicsWorld world_;
    BoxBody left_, right_, cube_;
    btSliderConstraint leftGuide_, rightGuide_;
};
}

int main(int argc, char** argv) {
    if ((argc != 2 && argc != 3) || std::string_view(argv[1]) != "shared8") return 2;
    int iterations = 1;
    if (argc == 3) {
        const std::string_view arg(argv[2]);
        const auto end = arg.data() + arg.size();
        const auto result = std::from_chars(arg.data(), end, iterations);
        if (result.ec != std::errc{} || result.ptr != end || iterations < 1) return 2;
    }
    grip::header();
    std::string name = sizeof(btScalar) == 8 ? "bullet-f64-shared8" : "bullet-f32-shared8";
    if (iterations != 1) name += "-i" + std::to_string(iterations);
    return grip::sweep(name.c_str(), [iterations](grip::Load load) { return Rig(load, iterations); }) ? 0 : 1;
}
