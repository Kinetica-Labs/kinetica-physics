// Собирать как потребителя цели Jolt: определения точности, набор инструкций и ABI общие с библиотекой.
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include "../studies/GripLimit.h"
#include <string_view>

namespace {
using namespace JPH;

class JoltRuntime {
public:
    JoltRuntime() {
        RegisterDefaultAllocator();
        Factory::sInstance = new Factory;
        RegisterTypes();
    }
    ~JoltRuntime() {
        UnregisterTypes();
        delete Factory::sInstance;
        Factory::sInstance = nullptr;
    }
    JoltRuntime(const JoltRuntime&) = delete;
};

class Rig {
public:
    explicit Rig(grip::Load load) {
        // Все три тела динамические и используют один слой. Обе направляющие закреплены в мире.
        world_.Init(16, 1, 64, 64, layer_, broadFilter_, pairFilter_);
        world_.SetGravity(Vec3::sZero());
        auto settings = world_.GetPhysicsSettings();
        settings.mNumVelocitySteps = settings.mNumPositionSteps = 1;
        settings.mConstraintWarmStart = true;
        settings.mAllowSleeping = false;
        world_.SetPhysicsSettings(settings);
        left_ = addBox(grip::jawPosition(grip::leftJaw, load.open), load.friction);
        right_ = addBox(grip::jawPosition(grip::rightJaw, load.open), load.friction);
        cube_ = addBox(grip::cube, load.friction);
        if (left_ && right_ && cube_) {
            leftGuide_ = guide(*left_);
            rightGuide_ = guide(*right_);
        }
    }
    ~Rig() {
        for (auto* slider : {rightGuide_, leftGuide_})
            if (slider) world_.RemoveConstraint(slider);
        for (auto* body : {cube_, right_, left_}) {
            if (!body) continue;
            const BodyID id = body->GetID();
            world_.GetBodyInterface().RemoveBody(id);
            world_.GetBodyInterface().DestroyBody(id);
        }
    }
    Rig(const Rig&) = delete;

    bool step(double dt, double squeeze, double down) {
        if (!leftGuide_ || !rightGuide_) return false;
        auto& bodies = world_.GetBodyInterface();
        for (int i = 0; i < 8; ++i) {
            bodies.AddForce(left_->GetID(), Vec3(float(squeeze), 0, 0));
            bodies.AddForce(right_->GetID(), Vec3(float(-squeeze), 0, 0));
            bodies.AddForce(cube_->GetID(), Vec3(0, float(-down), 0));
            if (world_.Update(float(dt / 8), 1, &scratch_, &jobs_) != EPhysicsUpdateError::None) return false;
        }
        return true;
    }
    grip::Reading read() const {
        const double jawY = (double(left_->GetPosition().GetY()) + right_->GetPosition().GetY()) / 2;
        return {cube_->GetPosition().GetY() - jawY, momentumX(*left_), momentumX(*right_)};
    }
private:
    Body* addBox(grip::Box b, double mu) {
        // Нулевой радиус скругления оставляет заданные плоские грани и острые рёбра.
        Ref<BoxShape> shape = new BoxShape(Vec3(float(b.halfX), float(b.halfY), float(b.halfZ)), 0);
        shape->SetDensity(float(b.density));
        BodyCreationSettings info(shape, RVec3(Real(b.x), 0, 0), Quat::sIdentity(), EMotionType::Dynamic, 0);
        info.mFriction = float(mu); // Jolt использует геометрическое среднее.
        info.mRestitution = info.mLinearDamping = info.mAngularDamping = 0;
        info.mAllowSleeping = false;
        info.mMotionQuality = EMotionQuality::Discrete;
        auto& bodies = world_.GetBodyInterface();
        Body* body = bodies.CreateBody(info);
        if (body) bodies.AddBody(body->GetID(), EActivation::Activate);
        return body;
    }
    Constraint* guide(Body& jaw) {
        SliderConstraintSettings settings;
        settings.mPoint1 = settings.mPoint2 = jaw.GetPosition();
        settings.SetSliderAxis(Vec3::sAxisX());
        auto* slider = settings.Create(Body::sFixedToWorld, jaw);
        world_.AddConstraint(slider);
        return slider;
    }
    static double momentumX(const Body& body) {
        return double(body.GetLinearVelocity().GetX()) / body.GetMotionProperties()->GetInverseMass();
    }
    BroadPhaseLayerInterfaceTable layer_{1, 1};
    ObjectVsBroadPhaseLayerFilter broadFilter_;
    ObjectLayerPairFilter pairFilter_;
    TempAllocatorImpl scratch_{8 * 1024 * 1024};
    JobSystemSingleThreaded jobs_{cMaxPhysicsJobs};
    PhysicsSystem world_;
    Body *left_ = nullptr, *right_ = nullptr, *cube_ = nullptr;
    Constraint *leftGuide_ = nullptr, *rightGuide_ = nullptr;
};
}

int main(int argc, char** argv) {
    if (argc != 2 || std::string_view(argv[1]) != "shared8") return 2;
    JoltRuntime runtime;
    grip::header();
    // Jolt сохраняет скорости и импульсы в float даже при двойной точности мировых координат.
    const char* name = sizeof(JPH::Real) == 8 ? "jolt-pos64-shared8" : "jolt-f32-shared8";
    return grip::sweep(name, [](grip::Load load) { return Rig(load); }) ? 0 : 1;
}
