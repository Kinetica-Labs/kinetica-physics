#include <math/StepMemory.h>
#include <math/SmallVector.h>
#include <collision/narrowphase/ContactPatch.h>

#include <collision/narrowphase/ContactCandidate.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace kp {
namespace {

// Доли отрезка from—to, на которых он пересекает границу плоского элемента, по возрастанию.
StepArray<Real> crossings(const Flat& flat, Vector3 from, Vector3 to) {
    StepArray<Real> found;
    const Vector3 along = to - from;
    if (flat.disc()) {
        // Проекция отрезка на плоскость круга: |q + t d|² = R².
        const Vector3 q = flat.flatten(from - flat.centre), d = flat.flatten(along);
        const Real a = d.dot(d), b = q.dot(d), c = q.dot(q) - flat.radius * flat.radius;
        if (a < kOnBoundary * kOnBoundary || b * b < a * c) return found;
        const Real root = sqrt(b * b - a * c);
        found = {(-b - root) / a, (-b + root) / a};
    } else {
        Real enter = 0, leave = 1;
        for (size_t k = 0; k < flat.corners.size(); ++k) {
            const Real d0 = flat.outside(k, from), d1 = flat.outside(k, to);
            if (d0 > kOnBoundary && d1 > kOnBoundary) return found;
            if (d0 > kOnBoundary) enter = std::max(enter, d0 / (d0 - d1));
            if (d1 > kOnBoundary) leave = std::min(leave, d0 / (d0 - d1));
        }
        if (enter <= leave) found = {enter, leave};
    }
    found.erase(std::remove_if(found.begin(), found.end(), [](Real t) { return t <= 0 || t >= 1; }), found.end());
    return found;
}

// Точка плоскости контура над точкой p плоского элемента — вдоль его нормали.
Vector3 lifted(const Flat& flat, const Outline& outline, Vector3 p) {
    return p + (outline.normal.dot(outline.points[0] - p) / outline.normal.dot(flat.normal)) * flat.normal;
}

// Пересечения сторон контура (отрезка или многоугольника) с границей плоского элемента.
template <class Add>
void addSideCrossings(const Flat& flat, const Outline& outline, Add&& add) {
    const size_t sides = outline.kind == Outline::Kind::Segment ? 1 : outline.points.size();
    for (size_t k = 0; k < sides; ++k) {
        const Vector3 from = outline.points[k], to = outline.points[(k + 1) % outline.points.size()];
        std::uint32_t id = 0x100u + 2 * std::uint32_t(k);
        for (Real t : crossings(flat, from, to)) add(from + t * (to - from), id++);
    }
}

// Пересечения окружности контура со сторонами грани. Окружность c + r (u cos φ + v sin φ) и плоскость
// стороны грани: A cos φ + B sin φ = C.
template <class Add>
void addCircleOverFace(const Flat& flat, const Outline& outline, Add&& add) {
    const Vector3 u = (1 / outline.radius) * (outline.points[0] - outline.centre), v = (1 / outline.radius) * (outline.points[1] - outline.centre);
    for (size_t k = 0; k < flat.corners.size(); ++k) {
        const Vector3 tail = flat.corners[k], out = (flat.corners[(k + 1) % flat.corners.size()] - tail).cross(flat.normal);
        const Real a = outline.radius * out.dot(u), b = outline.radius * out.dot(v), c = -out.dot(outline.centre - tail);
        const Real reach = a * a + b * b;
        if (reach <= c * c) continue;
        const Real root = sqrt(reach - c * c);
        for (int sign : {-1, 1}) {
            const Real cosine = (a * c - sign * b * root) / reach, sine = (b * c + sign * a * root) / reach;
            const Vector3 p = outline.centre + outline.radius * (cosine * u + sine * v);
            if (flat.contains(p)) add(p, 0x100u + 2 * std::uint32_t(k) + (sign > 0));
        }
    }
}

using Polynomial = std::array<Real, 5>; // от свободного члена до четвёртой степени

Real valueAt(const Polynomial& p, int degree, Real x) {
    Real result = p[size_t(degree)];
    while (degree > 0) result = x * result + p[size_t(--degree)];
    return result;
}

Real bracketedRoot(const Polynomial& p, int degree, Real left, Real right, bool negativeLeft) {
    for (int i = 0; i < std::numeric_limits<Real>::digits + 1; ++i) {
        const Real mid = (left + right) / 2, value = valueAt(p, degree, mid);
        if (value == 0) return mid;
        if ((value < 0) == negativeLeft) left = mid;
        else right = mid;
    }
    return (left + right) / 2;
}

// На [-1,1] корни производной делят полином на монотонные участки. На каждом корень
// либо меняет знак, либо совпадает с концом участка (в том числе при касании).
SmallVector<Real, 8> arcRoots(const Polynomial& p, int degree) {
    SmallVector<Real, 8> roots;
    while (degree > 0 && p[size_t(degree)] == 0) --degree;
    if (degree == 0) return roots;
    Polynomial derivative{};
    Real scale = abs(p[0]);
    for (int i = 1; i <= degree; ++i) {
        derivative[size_t(i - 1)] = Real(i) * p[size_t(i)];
        scale += abs(p[size_t(i)]);
    }
    auto stops = arcRoots(derivative, degree - 1);
    stops.push_back(1);
    const Real roundoff = 16 * kEpsilon * scale;
    Real left = -1, before = valueAt(p, degree, left);
    if (abs(before) <= roundoff) roots.push_back(left);
    for (Real right : stops) {
        if (right <= left) continue;
        const Real after = valueAt(p, degree, right);
        if (abs(before) > roundoff && abs(after) > roundoff && (before < 0) != (after < 0))
            roots.push_back(bracketedRoot(p, degree, left, right, before < 0));
        if (abs(after) <= roundoff) roots.push_back(right);
        left = right;
        before = after;
    }
    return roots;
}

// Полуокружность p(t) = centre + ((1-t²)u + 2tv)/(1+t²), -1 ≤ t ≤ 1.
// Подстановка её проекции в уравнение опорной окружности |p - c|² = R² даёт полином.
template <class Add>
void addTiltedRims(const Flat& flat, const Outline& outline, Add&& add) {
    const Vector3 centre = flat.flatten(outline.centre - flat.centre);
    const Real rSquared = flat.radius * flat.radius;
    std::uint32_t id = 0x100u;
    for (Real sign : {Real(1), Real(-1)}) {
        const Vector3 u = sign * (outline.points[0] - outline.centre), v = sign * (outline.points[1] - outline.centre);
        const Vector3 a = centre + flat.flatten(u), b = 2 * flat.flatten(v), c = centre - flat.flatten(u);
        const Polynomial p{a.dot(a) - rSquared, 2 * a.dot(b), b.dot(b) + 2 * a.dot(c) - 2 * rSquared,
                           2 * b.dot(c), c.dot(c) - rSquared};
        for (Real t : arcRoots(p, 4)) {
            if (t == 1) continue; // начало следующей полуокружности
            const Real tt = t * t;
            const Vector3 point = outline.centre + (1 / (1 + tt)) * ((1 - tt) * u + (2 * t) * v);
            if (abs(flat.flatten(point - flat.centre).length() - flat.radius) <= kOnBoundary) add(point, id++);
        }
    }
}

template <class Add>
void addCircleOverDisc(const Flat& flat, const Outline& outline, Add&& add) {
    const Vector3 between = flat.flatten(outline.centre - flat.centre);
    const Real d = between.length(), r1 = flat.radius, r2 = outline.radius;
    const Real cosine = abs(flat.normal.dot(outline.normal));
    if (d > r1 + r2 || d + r2 < r1 || d + r1 < r2 * cosine) return;
    if (cosine < 1 - 4 * kEpsilon) {
        addTiltedRims(flat, outline, add);
    } else if (d > kOnBoundary && d >= abs(r1 - r2)) {
        const Real a = (d * d + r1 * r1 - r2 * r2) / (2 * d), h = sqrt(std::max(Real(0), r1 * r1 - a * a));
        const Vector3 along = (1 / d) * between, across = flat.normal.cross(along);
        for (int sign : {1, -1}) {
            const Vector3 p = lifted(flat, outline, flat.centre + a * along + (sign * h) * across);
            if ((p - outline.centre).length() <= r2 + kOnBoundary)
                add(p, sign > 0 ? 0x100u : 0x101u);
        }
    }
}

// Углы плоского элемента под замкнутым контуром. У круга углы — четыре точки окружности, первая — под
// самым низким местом контура.
template <class Add>
void addCornersUnder(const Flat& flat, const Outline& outline, Add&& add) {
    const StepArray<Vector3> corners = flat.disc() ? Flat::quarterPoints(flat.centre, flat.normal, flat.radius, Flat::inPlane(-1 * outline.normal, flat.normal, flat.side)) : flat.corners;
    Flat contour; // сам контур как плоский элемент: лежит ли точка внутри него
    contour.normal = outline.normal, contour.centre = outline.centre, contour.radius = outline.radius;
    if (outline.kind == Outline::Kind::Loop) contour.corners = outline.points;
    for (size_t k = 0; k < corners.size(); ++k) {
        const Vector3 above = lifted(flat, outline, corners[k]);
        if (contour.contains(above)) add(above, 0x200u + std::uint32_t(k));
    }
}

} // namespace

StepArray<ContactPatch::Point> ContactPatch::pointsOf(const Flat& flat, const Outline& outline) {
    StepArray<Point> points;
    using Kind = Outline::Kind;
    // Одна и та же вершина площадки бывает получена дважды — как точка контура над элементом и как
    // угол элемента под контуром (два равных торца друг на друге). В контакт она идёт один раз.
    const auto add = [&](Vector3 core, std::uint32_t id) {
        for (const Point& known : points)
            if ((known.core - core).length() <= kOnBoundary) return;
        points.push_back({core, id});
    };
    // 1) Вершины контура над плоским элементом.
    for (size_t i = 0; i < outline.points.size(); ++i)
        if (flat.contains(outline.points[i])) add(outline.points[i], std::uint32_t(i));
    // 2) Пересечения сторон контура с границей плоского элемента.
    if (outline.kind == Kind::Segment || outline.kind == Kind::Loop) addSideCrossings(flat, outline, add);
    else if (outline.kind == Kind::Circle && !flat.disc()) addCircleOverFace(flat, outline, add);
    else if (outline.kind == Kind::Circle) addCircleOverDisc(flat, outline, add);
    // 3) Углы плоского элемента под замкнутым контуром.
    if (outline.closed() && abs(outline.normal.dot(flat.normal)) > Real(0.1)) addCornersUnder(flat, outline, add);
    return points;
}

std::optional<ContactPatch> ContactPatch::on(const ConvexShape& reference, const Transform& referencePose, const ConvexShape& other, const Transform& otherPose, Vector3 towards) {
    const std::optional<Flat> flat = reference.flatTowards(referencePose, towards);
    if (!flat) return std::nullopt;
    ContactPatch patch{*flat, other.outlineOver(otherPose, *flat), {}, 0};
    patch.points = pointsOf(patch.flat, patch.outline);
    const Vector3 lowest = otherPose.point(other.support(otherPose.localDirection(-1 * flat->normal)));
    patch.gap = flat->height(lowest) - flat->rounding - other.radius;
    return patch;
}

ContactManifold ContactPatch::manifold(bool onReferenceB, Real margin, Real touching) const {
    ContactManifold manifold;
    manifold.faceOnB = onReferenceB;
    manifold.normal = (onReferenceB ? Real(-1) : Real(1)) * flat.normal;
    StepArray<ContactPoint> found;
    ContactCandidates candidates;
    for (const Point& p : points) {
        const Real height = flat.height(p.core);
        const Vector3 onReference = p.core - (height - flat.rounding) * flat.normal, onOther = p.core - outline.rounding * flat.normal;
        const Real separation = height - flat.rounding - outline.rounding;
        if (separation > margin) continue;
        candidates.push_back({onReference, separation, std::uint32_t(found.size())});
        found.push_back({onReferenceB ? onOther : onReference, onReferenceB ? onReference : onOther, separation, p.id | (onReferenceB ? 0x1000u : 0u)});
    }
    for (const ContactCandidate& kept : ContactCandidate::ordered(candidates, touching)) manifold.add(found[kept.id]);
    return manifold;
}

ContactManifold ContactPatch::between(const ConvexShape& a, const Transform& poseA, const ConvexShape& b, const Transform& poseB, const ClosestPoints& closest, Real margin,
                              Real touching) {
    const std::optional<ContactPatch> onA = on(a, poseA, b, poseB, closest.normal), onB = on(b, poseB, a, poseA, -1 * closest.normal);
    const auto fits = [&](const std::optional<ContactPatch>& patch) { return patch && !patch->points.empty() && patch->gap >= closest.distance - std::max(touching, kOnBoundary); };
    const bool useB = fits(onB) && (!fits(onA) || onB->gap > onA->gap + kOnBoundary);
    if (!useB && !fits(onA)) return {};
    ContactManifold manifold = (useB ? *onB : *onA).manifold(useB, margin, touching);
    manifold.roundA = a.rollingRadius(poseA, manifold.normal), manifold.roundB = b.rollingRadius(poseB, -1 * manifold.normal);
    return manifold;
}

} // namespace kp
