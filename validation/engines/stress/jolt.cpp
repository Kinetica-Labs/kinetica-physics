// Jolt с однопоточной очередью заданий и тем же тактом, что у остальных движков.
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include "Stress1000.h"

namespace {
using namespace JPH;
template<class Vec> V3 point(Vec p) { return {p.GetX(),p.GetY(),p.GetZ()}; }

void simulate(const SceneSpec& s, const Play& play) {
    BroadPhaseLayerInterfaceTable layer(1,1);
    ObjectVsBroadPhaseLayerFilter broadFilter;
    ObjectLayerPairFilter pairFilter;
    TempAllocatorImpl scratch(64*1024*1024);
    JobSystemSingleThreaded jobs(cMaxPhysicsJobs);
    PhysicsSystem world;
    world.Init(2048,1,65536,32768,layer,broadFilter,pairFilter);
    world.SetGravity(Vec3(0,float(-kGravity),0));
    auto settings=world.GetPhysicsSettings();
    settings.mConstraintWarmStart=true;
    settings.mAllowSleeping=false;
    settings.mNumVelocitySteps=settings.mNumPositionSteps=1;
    world.SetPhysicsSettings(settings);
    const auto makeShape=[](const BoxSpec& b) {
        Ref<BoxShape> shape=new BoxShape(Vec3(float(b.half.x),float(b.half.y),float(b.half.z)),0);
        if (b.density>0) shape->SetDensity(float(b.density));
        return shape;
    };
    auto floor=makeShape(s.boxes.front()), cube=makeShape(s.boxes[1]);
    auto& api=world.GetBodyInterface();
    std::vector<Body*> bodies;
    for (const auto& b:s.boxes) {
        const bool dynamic=b.density>0;
        BodyCreationSettings info(dynamic?cube:floor,RVec3(Real(b.at.x),Real(b.at.y),Real(b.at.z)),Quat::sIdentity(),
                                  dynamic?EMotionType::Dynamic:EMotionType::Static,0);
        info.mFriction=.5f;
        info.mRestitution=info.mLinearDamping=info.mAngularDamping=0;
        info.mAllowSleeping=false;
        info.mLinearVelocity=Vec3(float(b.velocity.x),float(b.velocity.y),float(b.velocity.z));
        Body* body=api.CreateBody(info);
        stress::require(body!=nullptr,"Jolt could not create a body");
        api.AddBody(body->GetID(),dynamic?EActivation::Activate:EActivation::DontActivate);
        bodies.push_back(body);
    }
    world.OptimizeBroadPhase();
    play([&](double dt) {
        stress::require(world.Update(float(dt),1,&scratch,&jobs)==EPhysicsUpdateError::None,"incomplete Jolt update");
    },[&] {
        std::vector<State> result;
        result.reserve(bodies.size());
        for (const Body* b:bodies) {
            const auto q=b->GetRotation();
            result.push_back({point(b->GetPosition()),{q.GetW(),q.GetX(),q.GetY(),q.GetZ()},point(b->GetLinearVelocity()),
                              point(b->GetAngularVelocity()),!b->IsActive()});
        }
        return result;
    });
    for (auto* b:bodies) {
        const auto id=b->GetID();
        api.RemoveBody(id);
        api.DestroyBody(id);
    }
}
}

int main() {
    stress::require(stress::requested(),"set STRESS_SCENE and STRESS_PASS");
    JPH::RegisterDefaultAllocator();
    JPH::Factory::sInstance=new JPH::Factory;
    JPH::RegisterTypes();
    stress::compareOrStress("jolt-f32",simulate);
    JPH::UnregisterTypes();
    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance=nullptr;
}
