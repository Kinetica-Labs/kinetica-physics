// Адаптер общего опыта захвата. RP3D обновляет контакты и прикладывает силы каждый подшаг.
#include "../studies/GripLimit.h"
#include "../studies/GripSensitivity.h"
#include <vector> // у этой версии RP3D публичные заголовки используют std::vector
#include <string_view>
#include <reactphysics3d/reactphysics3d.h>
#include "RpWindowsAllocator.h"

namespace {
using namespace reactphysics3d;

class Rig {
public:
    Rig(grip::Load load, bool singlePass, decimal persistDistance = 0) {
        initialize(load, singlePass, persistDistance);
    }
#ifdef KP_RP3D_MEMORY_STUDY
    Rig(grip::Load load, bool singlePass, MemoryAllocator& base) : common_(&base) {
        initialize(load, singlePass, 0);
    }
#endif
private:
    void initialize(grip::Load load, bool singlePass, decimal persistDistance) {
        PhysicsWorld::WorldSettings settings;
        if (persistDistance > 0) settings.persistentContactDistanceThreshold = persistDistance;
        settings.gravity = Vector3(0, 0, 0);
        settings.isSleepingEnabled = false;
        if (singlePass) {
            settings.defaultVelocitySolverNbIterations = 1;
            settings.defaultPositionSolverNbIterations = 1;
        }
        world_ = common_.createPhysicsWorld(settings);
        auto* anchor = world_->createRigidBody(Transform::identity());
        anchor->setType(BodyType::STATIC);
        left_ = addBox(grip::jawPosition(grip::leftJaw, load.open), load.friction);
        right_ = addBox(grip::jawPosition(grip::rightJaw, load.open), load.friction);
        cube_ = addBox(grip::cube, load.friction);
        for (auto* jaw : {left_, right_}) {
            SliderJointInfo slider(anchor, jaw, jaw->getTransform().getPosition(), Vector3(1, 0, 0));
            slider.isCollisionEnabled = false;
            world_->createJoint(slider);
        }
    }
public:
    ~Rig() { common_.destroyPhysicsWorld(world_); }
    Rig(const Rig&) = delete;
    bool step(double dt, double squeeze, double down) {
        for (int i = 0; i < 8; ++i) {
            left_->applyWorldForceAtCenterOfMass(Vector3(decimal(squeeze), 0, 0));
            right_->applyWorldForceAtCenterOfMass(Vector3(-squeeze, 0, 0));
            cube_->applyWorldForceAtCenterOfMass(Vector3(0, decimal(-down), 0));
            world_->update(decimal(dt / 8));
        }
        return true;
    }
    grip::Reading read() const {
        const double jawY = (double(left_->getTransform().getPosition().y) + right_->getTransform().getPosition().y) / 2;
        return {cube_->getTransform().getPosition().y - jawY,
                double(left_->getMass()) * left_->getLinearVelocity().x,
                double(right_->getMass()) * right_->getLinearVelocity().x};
    }
private:
    RigidBody* addBox(grip::Box b, double mu) {
        auto* body = world_->createRigidBody(Transform(Vector3(decimal(b.x), 0, 0), Quaternion::identity()));
        auto* shape = common_.createBoxShape(Vector3(decimal(b.halfX), decimal(b.halfY), decimal(b.halfZ)));
        auto& material = body->addCollider(shape, Transform::identity())->getMaterial();
        material.setFrictionCoefficient(decimal(mu));
        material.setBounciness(0);
        material.setMassDensity(decimal(b.density));
        body->updateMassPropertiesFromColliders();
        return body;
    }
#ifdef _WIN32
    RpWindowsAllocator allocator_;
    PhysicsCommon common_{&allocator_};
#else
    PhysicsCommon common_;
#endif
    PhysicsWorld* world_;
    RigidBody *left_, *right_, *cube_;
};
}

#ifndef KP_RP3D_MEMORY_STUDY
int main(int argc, char** argv) {
    if (argc == 2 && std::string_view(argv[1]) == "sensitivity") {
        grip::sensitivityHeader();
        bool complete = true;
        for (double distance : {0.015, 0.030, 0.060})
            complete &= grip::sensitivity(sizeof(decimal) == 8 ? "rp3d-f64" : "rp3d-f32", "persistence_m", distance,
                                          [distance](grip::Load load) { return Rig(load, true, decimal(distance)); });
        return complete ? 0 : 1;
    }
    const bool shared = argc == 2 && std::string_view(argv[1]) == "shared8";
    if (argc != 1 && !shared) return 2;
    grip::header();
    const char* name = sizeof(decimal) == sizeof(double) ? "rp3d-f64" : "rp3d-f32";
    if (shared) name = sizeof(decimal) == sizeof(double) ? "rp3d-f64-shared8" : "rp3d-f32-shared8";
    return grip::sweep(name, [shared](grip::Load load) { return Rig(load, shared); }) ? 0 : 1;
}
#endif
