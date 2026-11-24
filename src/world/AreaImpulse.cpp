#include <world/AreaImpulse.h>

#include <collision/narrowphase/Gjk.h>
#include <collision/shapes/Sphere.h>

#include <limits>

namespace kp {
namespace {

// Ближайшая к центру шара точка тела и расстояние до неё; inside — центр внутри тела.
struct Nearest {
    Vector3 point;
    Real distance;
    bool inside;
};

Nearest nearestPoint(const Body& body, Vector3 center) {
    const Sphere probe(0);
    Nearest best{body.position, std::numeric_limits<Real>::infinity(), false};
    for (int p = 0; p < body.partCount(); ++p) {
        const Body part = body.part(p);
        if (!part.shape) continue;
        const ClosestPoints gap = closestPoints(probe, {center, {}}, *part.shape, part.pose());
        if (gap.coresOverlap || gap.distance <= 0) return {center, 0, true};
        if (gap.distance < best.distance) best = {gap.onB, gap.distance, false};
    }
    return best;
}

// Обходит подвижные тела, задетые шаром; push(тело, ближайшая точка, доля импульса) говорит, толкнул ли.
template <class Push> int pushWithin(World& world, Vector3 center, Real radius, int falloff, Push&& push) {
    int touched = 0;
    for (size_t i = 0; i < world.bodies().size(); ++i) {
        if (world.bodies()[i].isStatic()) continue;
        const Nearest nearest = nearestPoint(world.bodies()[i], center);
        if (!(nearest.distance < radius)) continue;
        Real share = 1;
        for (int k = 0; k < falloff; ++k) share *= 1 - nearest.distance / radius;
        world.wake(BodyId(int(i))); // спящее тело на время сна неподвижно
        touched += push(world.body(BodyId(int(i))), nearest, share) ? 1 : 0;
    }
    return touched;
}

} // namespace

// Центр взрыва внутри тела: ближайшей точки нет, импульс идёт в центр масс; совпал с ним — толкать некуда.
int applyRadialImpulse(World& world, Vector3 center, Real radius, Real impulse, int falloff) {
    return pushWithin(world, center, radius, falloff, [&](Body& body, const Nearest& nearest, Real share) {
        const Vector3 at = nearest.inside ? body.position : nearest.point, away = at - center;
        if (!(away.length() > 0)) return false;
        applyImpulse(body, (share * impulse / away.length()) * away, at);
        return true;
    });
}

int applyDirectedImpulse(World& world, Vector3 center, Real radius, Vector3 impulse, int falloff) {
    return pushWithin(world, center, radius, falloff, [&](Body& body, const Nearest&, Real share) {
        applyImpulse(body, share * impulse, body.position);
        return true;
    });
}

} // namespace kp
