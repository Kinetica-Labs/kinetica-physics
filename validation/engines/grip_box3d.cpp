// Адаптер общего опыта захвата. Box3D, восемь подшагов на кадр.
#include "../studies/GripLimit.h"
#include "../studies/GripSensitivity.h"
#include <box3d/box3d.h>
#include <string_view>

namespace {
class Rig {
public:
    Rig(grip::Load load, bool sharedClock, float contactHz = 0) : sharedClock_(sharedClock) {
        auto wd = b3DefaultWorldDef();
        if (contactHz > 0) wd.contactHertz = contactHz;
        wd.gravity = {0, 0, 0};
        wd.enableSleep = false;
        world_ = b3CreateWorld(&wd);
        b3World_EnableWarmStarting(world_, true);
        auto anchorDef = b3DefaultBodyDef();
        auto anchor = b3CreateBody(world_, &anchorDef);
        left_ = addBox(grip::jawPosition(grip::leftJaw, load.open), load.friction);
        right_ = addBox(grip::jawPosition(grip::rightJaw, load.open), load.friction);
        cube_ = addBox(grip::cube, load.friction);
        for (auto jaw : {left_, right_}) {
            auto slider = b3DefaultPrismaticJointDef();
            slider.base.bodyIdA = anchor;
            slider.base.bodyIdB = jaw;
            slider.base.localFrameA.p.x = float(b3Body_GetPosition(jaw).x);
            b3CreatePrismaticJoint(world_, &slider);
        }
    }
    ~Rig() { b3DestroyWorld(world_); }
    Rig(const Rig&) = delete;
    bool step(double dt, double squeeze, double down) {
        const int updates = sharedClock_ ? 8 : 1;
        for (int i = 0; i < updates; ++i) {
            b3Body_ApplyForceToCenter(left_, {float(squeeze), 0, 0}, true);
            b3Body_ApplyForceToCenter(right_, {-float(squeeze), 0, 0}, true);
            b3Body_ApplyForceToCenter(cube_, {0, -float(down), 0}, true);
            b3World_Step(world_, float(dt / updates), 8 / updates);
        }
        return true;
    }
    grip::Reading read() const {
        const double jawY = (double(b3Body_GetPosition(left_).y) + b3Body_GetPosition(right_).y) / 2;
        return {b3Body_GetPosition(cube_).y - jawY,
                double(b3Body_GetMass(left_)) * b3Body_GetLinearVelocity(left_).x,
                double(b3Body_GetMass(right_)) * b3Body_GetLinearVelocity(right_).x};
    }
private:
    b3BodyId addBox(grip::Box b, double mu) {
        auto body = b3DefaultBodyDef();
        body.type = b3_dynamicBody;
        body.position.x = float(b.x);
        auto id = b3CreateBody(world_, &body);
        auto shape = b3DefaultShapeDef();
        shape.density = float(b.density);
        shape.baseMaterial.friction = float(mu);
        shape.baseMaterial.restitution = 0;
        auto hull = b3MakeBoxHull(float(b.halfX), float(b.halfY), float(b.halfZ));
        b3CreateHullShape(id, &shape, &hull.base);
        return id;
    }
    b3WorldId world_;
    bool sharedClock_;
    b3BodyId left_, right_, cube_;
};
}

int main(int argc, char** argv) {
    if (argc == 2 && std::string_view(argv[1]) == "sensitivity") {
        grip::sensitivityHeader();
        bool complete = true;
        for (double hz : {30., 60.})
            complete &= grip::sensitivity("box3d", "contact_hz", hz, [hz](grip::Load load) { return Rig(load, true, float(hz)); });
        return complete ? 0 : 1;
    }
    const bool shared = argc == 2 && std::string_view(argv[1]) == "shared8";
    if (argc != 1 && !shared) return 2;
    grip::header();
    return grip::sweep(shared ? "box3d-shared8" : "box3d", [shared](grip::Load load) { return Rig(load, shared); }) ? 0 : 1;
}
