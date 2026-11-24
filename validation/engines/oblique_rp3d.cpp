// Геометрия и оракул общие у всех движков. Один update, один проход скорости и один положения.
#include "../studies/ObliqueGrip.h"
#include <vector>
#include <reactphysics3d/reactphysics3d.h>
#ifdef _WIN32
#include <malloc.h>
#endif

namespace {
using namespace oblique;
using namespace reactphysics3d;

#ifdef _WIN32
class AlignedAllocator final : public MemoryAllocator {
    void* allocate(size_t bytes) override { return _aligned_malloc(bytes, GLOBAL_ALIGNMENT); }
    void release(void* memory, size_t) override { _aligned_free(memory); }
};
#endif

class Clamp {
public:
    explicit Clamp(std::array<double, 2> mu) {
        PhysicsWorld::WorldSettings settings;
        settings.gravity = Vector3(0, 0, 0);
        settings.isSleepingEnabled = false;
        settings.defaultVelocitySolverNbIterations = 1;
        settings.defaultPositionSolverNbIterations = 1;
        world_ = common_.createPhysicsWorld(settings);
        auto* anchor = world_->createRigidBody(Transform::identity());
        anchor->setType(BodyType::STATIC);
        auto* jawShape = common_.createBoxShape(Vector3(decimal(jawHalfX), decimal(jawHalfYZ), decimal(jawHalfYZ)));
        for (int i : {0, 1}) {
            jaws_[i] = bodyWithShape(jawPosition(i), jawShape, jawDensity, mu[i] * mu[i]);
            SliderJointInfo guide(anchor, jaws_[i], jaws_[i]->getTransform().getPosition(), Vector3(1, 0, 0));
            guide.isCollisionEnabled = false;
            world_->createJoint(guide);
        }
        sphere_ = bodyWithShape(0, common_.createSphereShape(decimal(radius)), sphereDensity, 1);
    }
    ~Clamp() { common_.destroyPhysicsWorld(world_); }
    Clamp(const Clamp&) = delete;
    bool step() {
        jaws_[0]->applyWorldForceAtCenterOfMass(Vector3(decimal(force), 0, 0));
        jaws_[1]->applyWorldForceAtCenterOfMass(Vector3(decimal(-force), 0, 0));
        world_->update(decimal(h));
        return true;
    }
    void launch(oblique::Pair u) {
        const auto motion = initialMotion(u);
        sphere_->setLinearVelocity(Vector3(0, decimal(motion.centre.real()), decimal(motion.centre.imag())));
        sphere_->setAngularVelocity(Vector3(0, decimal(motion.angular.real()), decimal(motion.angular.imag())));
    }
    oblique::Pair poleVelocities() const {
        const auto v = sphere_->getLinearVelocity(), w = sphere_->getAngularVelocity();
        return poleSpeeds({v.y, v.z}, {w.y, w.z});
    }
    oblique::Pair jawVelocities() const {
        oblique::Pair result;
        for (int i : {0, 1}) {
            auto pole = sphere_->getTransform().getPosition();
            pole.x += decimal(i == 0 ? -radius : radius);
            const auto& jaw = *jaws_[i]; // центры масс симметричных губок совпадают с началом тела
            const auto arm = pole - jaw.getTransform().getPosition();
            const auto v = jaw.getLinearVelocity() + jaw.getAngularVelocity().cross(arm);
            result[i] = {v.y, v.z};
        }
        return result;
    }
    std::array<double, 2> jawMomentaX() const {
        return {double(jaws_[0]->getMass()) * jaws_[0]->getLinearVelocity().x,
                double(jaws_[1]->getMass()) * jaws_[1]->getLinearVelocity().x};
    }
private:
    RigidBody* bodyWithShape(double x, CollisionShape* shape, double density, double friction) {
        auto* body = world_->createRigidBody(Transform(Vector3(decimal(x), 0, 0), Quaternion::identity()));
        auto& surface = body->addCollider(shape, Transform::identity())->getMaterial();
        surface.setMassDensity(decimal(density));
        surface.setFrictionCoefficient(decimal(friction));
        surface.setBounciness(0);
        body->updateMassPropertiesFromColliders();
        return body;
    }
#ifdef _WIN32
    AlignedAllocator allocator_;
    PhysicsCommon common_{&allocator_};
#else
    PhysicsCommon common_;
#endif
    PhysicsWorld* world_;
    RigidBody* sphere_;
    std::array<RigidBody*, 2> jaws_;
};
}

int main(int argc, char** argv) { return oblique::run<Clamp>(argc, argv); }
