// Та же установка и дискретный эталон, что в study-oblique-grip. Один подшаг на вызов.
#include "../studies/ObliqueGrip.h"
#include <box3d/box3d.h>

namespace {
using namespace oblique;

class Clamp {
public:
    explicit Clamp(std::array<double, 2> mu) {
        auto settings = b3DefaultWorldDef();
        settings.gravity = {0, 0, 0};
        settings.enableSleep = false;
        world_ = b3CreateWorld(&settings);
        b3World_EnableWarmStarting(world_, true);
        auto anchorDef = b3DefaultBodyDef();
        auto anchor = b3CreateBody(world_, &anchorDef);
        for (int i : {0, 1}) {
            jaws_[i] = dynamicBody(jawPosition(i));
            auto shape = material(jawDensity, mu[i] * mu[i]);
            auto hull = b3MakeBoxHull(float(jawHalfX), float(jawHalfYZ), float(jawHalfYZ));
            b3CreateHullShape(jaws_[i], &shape, &hull.base);
            auto guide = b3DefaultPrismaticJointDef();
            guide.base.bodyIdA = anchor;
            guide.base.bodyIdB = jaws_[i];
            guide.base.localFrameA.p.x = float(jawPosition(i));
            b3CreatePrismaticJoint(world_, &guide);
        }
        sphere_ = dynamicBody(0);
        const b3Sphere ball{{0,0,0}, float(radius)};
        auto shape = material(sphereDensity, 1);
        b3CreateSphereShape(sphere_, &shape, &ball);
    }
    ~Clamp() { b3DestroyWorld(world_); }
    Clamp(const Clamp&) = delete;
    bool step() {
        b3Body_ApplyForceToCenter(jaws_[0], {float(force), 0, 0}, true);
        b3Body_ApplyForceToCenter(jaws_[1], {-float(force), 0, 0}, true);
        b3World_Step(world_, float(h), 1);
        return true;
    }
    void launch(Pair u) {
        const auto motion = initialMotion(u);
        b3Body_SetLinearVelocity(sphere_, {0, float(motion.centre.real()), float(motion.centre.imag())});
        b3Body_SetAngularVelocity(sphere_, {0, float(motion.angular.real()), float(motion.angular.imag())});
    }
    Pair poleVelocities() const {
        const auto v = b3Body_GetLinearVelocity(sphere_), w = b3Body_GetAngularVelocity(sphere_);
        return poleSpeeds({v.y, v.z}, {w.y, w.z});
    }
    Pair jawVelocities() const {
        Pair result;
        for (int i : {0, 1}) {
            auto pole = b3Body_GetPosition(sphere_);
            pole.x += float(i == 0 ? -radius : radius);
            const auto v = b3Body_GetWorldPointVelocity(jaws_[i], pole);
            result[i] = {v.y, v.z};
        }
        return result;
    }
    std::array<double, 2> jawMomentaX() const {
        return {double(b3Body_GetMass(jaws_[0])) * b3Body_GetLinearVelocity(jaws_[0]).x,
                double(b3Body_GetMass(jaws_[1])) * b3Body_GetLinearVelocity(jaws_[1]).x};
    }
private:
    b3BodyId dynamicBody(double x) {
        auto body = b3DefaultBodyDef();
        body.type = b3_dynamicBody;
        body.position.x = float(x);
        return b3CreateBody(world_, &body);
    }
    static b3ShapeDef material(double density, double friction) {
        auto shape = b3DefaultShapeDef();
        shape.density = float(density);
        shape.baseMaterial.friction = float(friction);
        shape.baseMaterial.restitution = 0;
        return shape;
    }
    b3WorldId world_;
    b3BodyId sphere_;
    std::array<b3BodyId, 2> jaws_;
};
}

int main(int argc, char** argv) { return oblique::run<Clamp>(argc, argv); }
