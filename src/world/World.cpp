#include <world/World.h>

#include <collision/PairFinder.h>
#include <math/StepMemory.h>

#include <algorithm>

namespace kp {

// Тихий кадр: наибольший путь точки любого тела за dt при нынешних скоростях не больше допуска
// soft.contactDrift, с которым площадка пары и так идёт за телами (ManifoldMemory).
bool World::quiet(Real dt) const {
    for (const Body& body : bodies_)
        if (!body.isStatic() && (body.linearVelocity().length() + body.angularVelocity().length() * body.boundingRadius()) * dt > soft.contactDrift) return false;
    return true;
}

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
        // В тихом кадре контакты ищутся один раз: четыре поиска нужны при ударах, а в покое не меняют ни
        // осадку, ни перекрытие и стоят трети кадра (замер H69).
        const int updates = quiet(dt) ? 1 : soft.contactUpdates;
        part.restSpeed = soft.contactDrift / dt; // тело стоит, если его точка не пройдёт за кадр больше допуска
        part.substeps = std::max(1, soft.substeps / updates);
        for (int update = 0; update < updates; ++update) {
            PairFinder finder(bodies_, gravity);
            finder.boundary = soft.edgeSlop, finder.memory = &manifolds_;
            finder.collect(dt / updates, joints_, pairs_);
            notify(watch, StepStage::Pairs);
            part.step(bodies_, pairs_, joints_, gravity, dt / updates, contactForces_, jointImpulses_);
        }
        report_.segments = part.substeps * updates;
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
