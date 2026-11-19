#include <world/World.h>

#include <collision/PairFinder.h>

#include <algorithm>

namespace kp {

void World::step(Real dt) {
    report_ = {};
    sleep_.track(bodies_.size());
    sleep_.wakeTouched(bodies_, joints_, gravity, dt, sleeping);
    if (realtime) {
        SoftStep part = soft;
        part.substeps = std::max(1, soft.substeps / soft.contactUpdates);
        for (int update = 0; update < soft.contactUpdates; ++update) {
            PairFinder finder(bodies_, gravity);
            finder.boundary = soft.edgeSlop;
            part.step(bodies_, finder.pairs(dt / soft.contactUpdates, joints_), joints_, gravity, dt / soft.contactUpdates, contactForces_, jointImpulses_);
        }
        report_.segments = part.substeps * soft.contactUpdates;
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
