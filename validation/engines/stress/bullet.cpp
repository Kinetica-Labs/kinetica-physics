// Однопоточный Bullet, одна итерация последовательных импульсов на внешний вызов.
#include "Stress1000.h"
#include <btBulletDynamicsCommon.h>
#include <memory>

namespace {
btVector3 vec(V3 p) { return {btScalar(p.x),btScalar(p.y),btScalar(p.z)}; }
V3 readVec(const btVector3& p) { return {double(p.x()),double(p.y()),double(p.z())}; }

void simulate(const SceneSpec& s, const Play& play) {
    btDefaultCollisionConfiguration configuration;
    btCollisionDispatcher dispatcher(&configuration);
    btDbvtBroadphase broadphase;
    btSequentialImpulseConstraintSolver solver;
    btDiscreteDynamicsWorld world(&dispatcher,&broadphase,&solver,&configuration);
    world.setGravity(vec({0,-kGravity,0}));
    world.getSolverInfo().m_numIterations = 1;
    world.getSolverInfo().m_solverMode |= SOLVER_USE_WARMSTARTING;
    btBoxShape floor(vec(s.boxes.front().half)), cube(vec(s.boxes[1].half));
    std::vector<std::unique_ptr<btRigidBody>> bodies;
    for (const auto& box : s.boxes) {
        const auto mass = btScalar(massOf(box));
        auto* shape = mass == 0 ? &floor : &cube;
        btVector3 inertia(0,0,0);
        if (mass != 0) shape->calculateLocalInertia(mass,inertia);
        btRigidBody::btRigidBodyConstructionInfo info(mass,nullptr,shape,inertia);
        info.m_startWorldTransform.setIdentity();
        info.m_startWorldTransform.setOrigin(vec(box.at));
        info.m_friction = btScalar(std::sqrt(.5)); // произведение даёт парный коэффициент 0,5
        info.m_restitution = info.m_linearDamping = info.m_angularDamping = 0;
        auto b = std::make_unique<btRigidBody>(info);
        b->setActivationState(DISABLE_DEACTIVATION);
        b->setLinearVelocity(vec(box.velocity));
        world.addRigidBody(b.get());
        bodies.push_back(std::move(b));
    }
    play([&](double dt) {
        const auto h = btScalar(dt);
        stress::require(world.stepSimulation(h,1,h)==1, "unfinished Bullet step");
    }, [&] {
        std::vector<State> result;
        result.reserve(bodies.size());
        for (const auto& b : bodies) {
            const auto q=b->getOrientation();
            result.push_back({readVec(b->getCenterOfMassPosition()),{q.w(),q.x(),q.y(),q.z()},
                              readVec(b->getLinearVelocity()),readVec(b->getAngularVelocity()),!b->isActive()});
        }
        return result;
    });
    for (auto& b : bodies) world.removeRigidBody(b.get());
}
}

int main() {
    stress::require(stress::requested(), "set STRESS_SCENE and STRESS_PASS");
    stress::compareOrStress("bullet-f64", simulate);
}
