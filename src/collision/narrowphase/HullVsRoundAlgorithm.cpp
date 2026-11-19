#include <collision/narrowphase/HullVsRoundAlgorithm.h>

#include <collision/narrowphase/ContactPatch.h>

#include <limits>
#include <utility>

namespace kp {

ContactManifold HullVsRoundAlgorithm::collide(const ConvexShape& a, const Transform& poseA, const ConvexShape& b, const Transform& poseB, const ClosestPoints& closest, Real margin,
                                              Real touching, Real) const {
    if (!closest.coresOverlap && closest.distance > margin) return {};
    // У капсулы и цилиндра контактов с гранью несколько, а глубина известна и тогда, когда ядра
    // перекрылись.
    if (const ConvexHull* hull = a.hull()) return against(*hull, poseA, b, poseB, closest, margin, touching);
    // Многогранник — тело B: контакт строится от него и переворачивается.
    ContactManifold manifold = against(*b.hull(), poseB, a, poseA, {closest.distance, closest.onB, closest.onA, -1 * closest.normal, closest.coresOverlap, closest.iterations}, margin, touching);
    manifold.normal = -1 * manifold.normal;
    manifold.faceOnB = !manifold.faceOnB;
    std::swap(manifold.roundA, manifold.roundB);
    for (int i = 0; i < manifold.count; ++i) std::swap(manifold.points[i].onA, manifold.points[i].onB);
    return manifold;
}

ContactManifold HullVsRoundAlgorithm::against(const ConvexHull& hull, const Transform& hullPose, const ConvexShape& round, const Transform& roundPose, const ClosestPoints& closest,
                                              Real margin, Real touching) {
    ContactManifold single;
    if (!closest.coresOverlap) {
        const ContactManifold manifold = ContactPatch::between(hull, hullPose, round, roundPose, closest, margin, touching);
        if (manifold.count > 0) return manifold;
        // Касание ребром или вершиной многогранника: контакт один — ближайшие точки.
        single.normal = closest.normal;
        single.add({closest.onA, closest.onB, closest.distance, 0});
        single.roundB = round.rollingRadius(roundPose, -1 * closest.normal);
        return single;
    }
    // Ядра перекрылись: расстояния и направления нет. Берётся грань, вдоль нормали которой форма
    // вошла в многогранник меньше всего, — разделяющая ось по одним нормалям граней.
    int face = 0;
    Real best = -std::numeric_limits<Real>::infinity();
    Vector3 deepest;
    for (int f = 0; f < int(hull.faces().size()); ++f) {
        const Vector3 normal = hullPose.direction(hull.faces()[size_t(f)].normal);
        const Vector3 point = roundPose.point(round.support(roundPose.localDirection(-1 * normal)));
        const Real height = normal.dot(point - hullPose.point(hull.vertices()[size_t(hull.faces()[size_t(f)].loop[0])]));
        if (height > best) best = height, face = f, deepest = point;
    }
    ContactPatch patch{hull.faceFlat(hullPose, face), {}, {}, 0};
    patch.outline = round.outlineOver(roundPose, patch.flat);
    patch.points = ContactPatch::pointsOf(patch.flat, patch.outline);
    ContactManifold manifold = patch.manifold(false, margin, touching);
    manifold.roundB = single.roundB = round.rollingRadius(roundPose, -1 * patch.flat.normal);
    if (manifold.count > 0) return manifold;
    single.normal = patch.flat.normal;
    single.add({deepest - best * patch.flat.normal, deepest - round.radius * patch.flat.normal, best - round.radius, 0});
    return single;
}

} // namespace kp
