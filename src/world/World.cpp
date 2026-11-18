#include <kp/world/World.h>

#include <kp/collision/PairFinder.h>

namespace kp {

void World::step(Real dt) {
    report_ = {};
    sleep_.track(bodies_.size());
    sleep_.wakeTouched(bodies_, joints_, gravity, dt, sleeping);
    if (realtime) {
        PairFinder finder(bodies_, gravity);
        finder.boundary = soft.edgeSlop;
        soft.step(bodies_, finder.pairs(dt, joints_), joints_, gravity, dt, contactForces_, jointImpulses_);
        report_.segments = soft.substeps;
    } else {
        ExactStep exact(bodies_, joints_, gravity, contactForces_, stopped_, report_);
        exact.shockPropagation = shockPropagation, exact.maxSegments = maxSegments;
        if (observer) exact.observer = [this](StepStage stage) { observer->after(stage, *this); };
        exact.run(dt);
    }
    if (sleeping) sleep_.settle(bodies_, joints_, gravity, dt, sleepSpeed, sleepTime);
    report_.asleep = sleep_.count();
}

} // namespace kp
