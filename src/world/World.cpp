#include <world/World.h>

#include <collision/PairFinder.h>
#include <math/StepMemory.h>

#include <algorithm>

namespace kp {

void World::step(Real dt) {
    const StepMemory memory; // рабочие массивы шага — из запаса потока (math/StepMemory.h)
    report_ = {};
    sleep_.track(bodies_.size());
    sleep_.wakeTouched(bodies_, joints_, gravity, dt, sleeping);
    const StageObserver watch = observer ? StageObserver([this](StepStage stage) { observer->after(stage, *this); }) : StageObserver();
    if (realtime) {
        SoftStep part = soft;
        part.observer = watch;
        manifolds_.tolerance = soft.contactDrift;
        part.substeps = std::max(1, soft.substeps / soft.contactUpdates);
        for (int update = 0; update < soft.contactUpdates; ++update) {
            PairFinder finder(bodies_, gravity);
            finder.boundary = soft.edgeSlop, finder.memory = &manifolds_;
            const std::vector<ContactPair> pairs = finder.pairs(dt / soft.contactUpdates, joints_);
            notify(watch, StepStage::Pairs);
            part.step(bodies_, pairs, joints_, gravity, dt / soft.contactUpdates, contactForces_, jointImpulses_);
        }
        report_.segments = part.substeps * soft.contactUpdates;
    } else {
        ExactStep exact(bodies_, joints_, gravity, contactForces_, stopped_, report_);
        exact.shockPropagation = shockPropagation, exact.maxSegments = maxSegments;
        exact.observer = watch;
        exact.run(dt);
    }
    if (sleeping) sleep_.settle(bodies_, joints_, gravity, dt, sleepSpeed, sleepTime);
    report_.asleep = sleep_.count();
}

} // namespace kp
