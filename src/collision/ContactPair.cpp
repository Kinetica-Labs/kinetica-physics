#include <collision/ContactPair.h>

#include <collision/narrowphase/Gjk.h>
#include <collision/narrowphase/NarrowPhaseAlgorithm.h>
#include <collision/narrowphase/Sat.h>

#include <algorithm>

namespace kp {

Real maxPointSpeed(const Body& body) {
    if (body.isStatic()) return 0;
    const Real moving = body.linearVelocity().length();
    if (!body.shape) return moving;
    const Vector3 inverse = body.mass.inverseInertia;
    const Real momentum = body.angularMomentum.length();
    // Первая оценка: угловая скорость не больше |L| / наименьший момент инерции, а точка не дальше
    // охватывающего радиуса.
    const Real round = momentum * std::max({inverse.x, inverse.y, inverse.z}) * body.boundingRadius();
    if (body.placed) return moving + round; // форма стоит не в осях тела: оценка по главным осям к ней не относится
    // Вторая — по главным осям. Скорость точки от вращения — |ω × r|, и она не больше
    // Σ |ωₖ| ρₖ, где ρₖ — расстояние точки от оси k, а ωₖ = Lₖ / Iₖ. Составляющие L в осях тела
    // меняются с прецессией, но их сумма квадратов равна |L|², поэтому сумма не больше
    // |L| √Σ (ρₖ / Iₖ)². У вытянутого тела быстрое вращение возможно только вокруг длинной оси, а
    // от неё точки близко: у стержня эта оценка на порядок точнее первой.
    const ConvexShape& shape = *body.shape;
    const auto extent = [&](Vector3 axis) { return std::max(axis.dot(shape.support(axis)), -axis.dot(shape.support(-1 * axis))) + shape.radius; };
    const Vector3 e{extent({1, 0, 0}), extent({0, 1, 0}), extent({0, 0, 1})};
    const Real x = sqrt(e.y * e.y + e.z * e.z) * inverse.x, y = sqrt(e.x * e.x + e.z * e.z) * inverse.y, z = sqrt(e.x * e.x + e.y * e.y) * inverse.z;
    const Real axial = momentum * sqrt(x * x + y * y + z * z);
    return moving + std::min(round, axial);
}

namespace {

// Точки контакта пары — способом, который зависит от вида форм (NarrowPhaseAlgorithm).
ContactManifold manifoldOf(const Body& a, const Body& b, const ClosestPoints& gap, Real margin, Real boundary) {
    return NarrowPhaseAlgorithm::between(*a.shape, *b.shape).collide(*a.shape, a.pose(), *b.shape, b.pose(), gap, margin, kContactDistance, boundary);
}

} // namespace

Real overlapDepth(const Body& a, const Body& b) { return -separationOf(a, b).distance; }

Separation separationOf(const Body& a, const Body& b) {
    const ConvexHull* hullA = a.shape->hull();
    const ConvexHull* hullB = b.shape->hull();
    if (hullA && hullB) {
        const SeparatingAxis axis = findSeparatingAxis(*hullA, a.pose(), *hullB, b.pose());
        // Тело вошло в призму рельефа: глубина меряется от её наружной грани, а не от боковой,
        // через которую оно «вошло» на шве между призмами.
        if (SeparatingAxis outer; axis.separation < 0 && outerFaceAxis(TransformedHull(*hullA, a.pose()), TransformedHull(*hullB, b.pose()), outer))
            return {outer.separation, outer.normal};
        return {axis.separation, axis.normal};
    }
    const ClosestPoints gap = closestPoints(*a.shape, a.pose(), *b.shape, b.pose());
    if (!gap.coresOverlap) return {gap.distance, gap.normal};
    if (!hullA && !hullB) {
        const Vector3 between = b.pose().position - a.pose().position;
        return {-(a.shape->radius + b.shape->radius + 2 * kOverlap), between.length() > 0 ? (1 / between.length()) * between : Vector3{0, 1, 0}};
    }
    const ContactManifold manifold = manifoldOf(a, b, gap, 0, kOnBoundary);
    Real depth = 0;
    for (int i = 0; i < manifold.count; ++i) depth = std::max(depth, -manifold.points[i].separation);
    return {-depth, manifold.normal};
}

Real gapAlong(const Body& a, const Body& b, Vector3 n) {
    const Vector3 onA = a.pose().point(a.shape->support(a.pose().localDirection(n)));
    const Vector3 onB = b.pose().point(b.shape->support(b.pose().localDirection(-1 * n)));
    return n.dot(onB - onA) - a.shape->radius - b.shape->radius;
}

std::optional<ContactPair> examinePair(const Body& a, const Body& b, Vector3 gravity, Real h, Real boundary) {
    if (!a.shape || !b.shape || (a.isStatic() && b.isStatic())) return std::nullopt;
    return examinePairWithin(a, b, approachReach(maxPointSpeed(a), maxPointSpeed(b), gravity, h), boundary);
}

std::optional<ContactPair> examinePairWithin(const Body& a, const Body& b, Real reach, Real boundary) {
    if (!a.shape || !b.shape || (a.isStatic() && b.isStatic())) return std::nullopt;
    if ((b.pose().position - a.pose().position).length() - a.shape->boundingRadius() - b.shape->boundingRadius() > reach + kContactDistance) return std::nullopt;
    const ClosestPoints gap = closestPoints(*a.shape, a.pose(), *b.shape, b.pose());
    const Real distance = gap.coresOverlap ? 0 : gap.distance;
    if (distance > reach + kContactDistance) return std::nullopt;
    ContactPair pair;
    pair.distance = distance, pair.reach = reach, pair.manifold = manifoldOf(a, b, gap, kContactDistance + reach, boundary);
    return pair;
}

std::optional<ContactPair> examinePair(const std::vector<Body>& bodies, int i, int j, Vector3 gravity, Real h, Real boundary) {
    std::optional<ContactPair> pair = examinePair(bodies[size_t(i)], bodies[size_t(j)], gravity, h, boundary);
    if (pair) pair->a = i, pair->b = j;
    return pair;
}

} // namespace kp
