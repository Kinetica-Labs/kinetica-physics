// ReactPhysics3D на общих сценах (common.h). В сборку и табло не входит: на всех шести сценах он
// хуже Box3D даже с 20 итерациями (замер 2026-11-18). Исходники — research/refs/reactphysics3d:
//   cmake -S research/refs/reactphysics3d -B build/rp3d -DCMAKE_BUILD_TYPE=Release && cmake --build build/rp3d
//   g++ -O3 -DNDEBUG -std=c++17 -Iresearch/refs/reactphysics3d/include validation/engines/rp3d.cpp \
//       build/rp3d/libreactphysics3d.a -lpthread -o build/rp3d/compare-rp3d
//
//   compare-rp3d [ИТЕРАЦИЙ]     по умолчанию — умолчание RP3D
#include "common.h"
#include <reactphysics3d/reactphysics3d.h>

#include <cstdlib>

using namespace reactphysics3d;

static Vector3 toRp3d(V3 v) { return Vector3(decimal(v.x), decimal(v.y), decimal(v.z)); }

static RigidBody* addBox(PhysicsCommon& common, PhysicsWorld* world, const BoxSpec& b) {
    RigidBody* body = world->createRigidBody(Transform(toRp3d(b.at), Quaternion(decimal(b.turn.x), decimal(b.turn.y), decimal(b.turn.z), decimal(b.turn.w))));
    body->setType(b.density == 0 ? BodyType::STATIC : BodyType::DYNAMIC);
    Collider* collider = body->addCollider(common.createBoxShape(toRp3d(b.half)), Transform::identity());
    collider->getMaterial().setFrictionCoefficient(decimal(0.5));
    collider->getMaterial().setBounciness(0);
    collider->getMaterial().setMassDensity(decimal(b.density == 0 ? 1000 : b.density));
    body->updateMassPropertiesFromColliders();
    body->setLinearDamping(0), body->setAngularDamping(0);
    if (b.density != 0) body->setLinearVelocity(toRp3d(b.velocity));
    return body;
}

int main(int argc, char** argv) {
    const int iterations = argc > 1 ? std::atoi(argv[1]) : 0;
    compare(iterations ? "rp3d " + std::to_string(iterations) : "rp3d", [&](const SceneSpec& s, const Play& play) {
        PhysicsCommon common;
        PhysicsWorld::WorldSettings settings;
        settings.gravity = Vector3(0, decimal(-kGravity), 0);
        settings.isSleepingEnabled = false;
        if (iterations) settings.defaultVelocitySolverNbIterations = settings.defaultPositionSolverNbIterations = iterations;
        PhysicsWorld* world = common.createPhysicsWorld(settings);
        std::vector<RigidBody*> bodies;
        for (const BoxSpec& b : s.boxes) bodies.push_back(addBox(common, world, b));
        for (const JointSpec& j : s.joints) {
            if (j.weld) {
                FixedJointInfo info(bodies[size_t(j.a)], bodies[size_t(j.b)], toRp3d(j.anchor));
                info.isCollisionEnabled = false;
                world->createJoint(info);
            } else {
                HingeJointInfo info(bodies[size_t(j.a)], bodies[size_t(j.b)], toRp3d(j.anchor), Vector3(0, 0, 1));
                info.isCollisionEnabled = false;
                world->createJoint(info);
            }
        }
        play([&](double dt) { world->update(decimal(dt)); }, [&] {
            std::vector<State> st;
            for (RigidBody* body : bodies) {
                const Transform& t = body->getTransform();
                const Vector3 p = t.getPosition(), v = body->getLinearVelocity(), w = body->getAngularVelocity();
                const Quaternion q = t.getOrientation();
                st.push_back({{p.x, p.y, p.z}, {q.w, q.x, q.y, q.z}, {v.x, v.y, v.z}, {w.x, w.y, w.z}, body->isSleeping()});
            }
            return st; });
        common.destroyPhysicsWorld(world);
    });
}
