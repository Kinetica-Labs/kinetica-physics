// Опорные элементы базовых форм. Сборка с заголовками и статической библиотекой SDK.
// Каждая строка stdout — полный вызов closestPoints + collide, даже при отказе оракула.
#include <collision/ContactPair.h>
#include <collision/narrowphase/NarrowPhaseAlgorithm.h>
#include <collision/shapes/Capsule.h>
#include <collision/shapes/ConvexHull.h>
#include <collision/shapes/Cylinder.h>
#include <collision/shapes/Sphere.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

namespace {
using namespace kp;
constexpr double tolerance = 2e-7;    // м
constexpr double directionTol = 2e-4; // |n_native − n_oracle| у разделённых тел
constexpr double margin = .0001;      // м
constexpr double na = std::numeric_limits<double>::quiet_NaN();

enum class Family { ball, capsule, cylinder, box };

// Размеры аналитической формы заданы прибором. Функции оракула не вызывают support/GJK/SAT.
struct Model {
    Family family;
    const ConvexShape& shape;
    double radius, halfHeight, rounding;
    Vector3 half{};

    double projection(Vector3 n) const { // h(n), единичное направление в осях формы
        switch (family) {
        case Family::ball: return radius;
        case Family::capsule: return radius + halfHeight * std::abs(n.y);
        case Family::cylinder:
            return rounding + (radius - rounding) * std::hypot(n.x, n.z) +
                   (halfHeight - rounding) * std::abs(n.y);
        case Family::box:
            return half.x * std::abs(n.x) + half.y * std::abs(n.y) + half.z * std::abs(n.z);
        }
        return na;
    }

    double signedSurfaceDistance(Vector3 p) const {
        const double radial = std::hypot(p.x, p.z);
        if (family == Family::ball) return p.length() - radius;
        if (family == Family::capsule)
            return std::hypot(radial, std::max(0., std::abs(p.y) - halfHeight)) - radius;
        if (family == Family::cylinder) {
            const double r = radial - (radius - rounding), h = std::abs(p.y) - (halfHeight - rounding);
            return std::hypot(std::max(r, 0.), std::max(h, 0.)) + std::min(std::max(r, h), 0.) - rounding;
        }
        const double x = std::abs(p.x) - half.x, y = std::abs(p.y) - half.y, z = std::abs(p.z) - half.z;
        return std::hypot(std::max(x, 0.), std::max(y, 0.), std::max(z, 0.)) + std::min(std::max({x, y, z}), 0.);
    }
};

struct Feature {
    const char* name;
    const Model& model;
    Vector3 normal, point;
};

Vector3 unit(Vector3 v) { return (1 / v.length()) * v; }
const Vector3 radial{.8, 0, .6}, up{0, 1, 0};
const Vector3 axis{.36, .48, .8};

// На скруглённом ободе точка принадлежит окружности ядра плюс rho*n.
void cylinderFeatures(std::vector<Feature>& out, const Model& m, const std::array<const char*, 4>& names) {
    const Vector3 rim = (m.radius - m.rounding) * radial + (m.halfHeight - m.rounding) * up;
    out.push_back({names[0], m, up, .3 * (m.radius - m.rounding) * radial + m.halfHeight * up});
    out.push_back({names[1], m, radial, m.radius * radial - .2 * m.halfHeight * up});
    const Vector3 capward = .6 * radial + .8 * up, sideward = .8 * radial + .6 * up;
    out.push_back({names[2], m, capward, rim + m.rounding * capward});
    out.push_back({names[3], m, sideward, rim + m.rounding * sideward});
}

// Ни одно направление прибора не антипараллельно ±axis.
Quaternion align(Vector3 from, Vector3 to) {
    const Vector3 v = from.cross(to);
    return Quaternion{1 + from.dot(to), v.x, v.y, v.z}.unit();
}

Transform place(const Feature& f, bool second, const Transform& frame, double gap) {
    const Vector3 target = second ? -axis : axis;
    const Quaternion twist = Quaternion::fromRotationVector((second ? .73 : -.18) * axis);
    const Quaternion rotation = (frame.orientation * twist * align(f.normal, target)).unit();
    const Vector3 anchor = frame.point(Vector3{.125, -.25, .375} + (second ? gap : 0.) * axis);
    return {anchor - rotation.rotate(f.point), rotation};
}

// Плоскости и точки дают независимые границы: g <= dist(A,B) <= |b0−a0| = g при g>=0.
double planeGap(const Model& a, const Transform& pa, const Model& b, const Transform& pb, Vector3 n) {
    return (pb.position - pa.position).dot(n) -
           a.projection(pa.localDirection(n)) - b.projection(pb.localDirection(-n));
}

bool finite(Vector3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
double surfaceError(const Model& a, const Transform& pa, const Model& b, const Transform& pb, Vector3 onA, Vector3 onB) {
    return std::max(std::abs(a.signedSurfaceDistance(pa.localPoint(onA))),
                    std::abs(b.signedSurfaceDistance(pb.localPoint(onB))));
}

bool observe(const Feature& a, const Transform& pa, const Feature& b, const Transform& pb, Vector3 n,
             const char* frame, int swap, double gap) {
    const auto& ma = a.model;
    const auto& mb = b.model;
    const double lower = planeGap(ma, pa, mb, pb, n);
    const Vector3 a0 = pa.point(a.point), b0 = pb.point(b.point);
    const double upper = (b0 - a0).length();
    const bool oracle = std::abs(lower - gap) <= 1e-12 && std::abs(upper - gap) <= 1e-12 &&
                        surfaceError(ma, pa, mb, pb, a0, b0) <= 1e-12 &&
                        (pa.direction(a.normal) - n).length() <= 1e-12 && (pb.direction(b.normal) + n).length() <= 1e-12;

    const auto closest = closestPoints(ma.shape, pa, mb.shape, pb);
    const bool resolved = !closest.coresOverlap && !closest.beyond;
    double distance = na, closestSurface = na, closestVector = na, closestDirection = na;
    bool closestOk = gap == 0 && ma.family == Family::box && mb.family == Family::box && closest.coresOverlap;
    if (resolved) {
        distance = closest.distance;
        closestSurface = surfaceError(ma, pa, mb, pb, closest.onA, closest.onB);
        closestVector = (closest.onB - closest.onA - distance * closest.normal).length();
        closestDirection = (closest.normal - n).length();
        closestOk = finite(closest.normal) && finite(closest.onA) && finite(closest.onB) &&
                    std::abs(closest.normal.length() - 1) <= directionTol && std::abs(distance - gap) <= tolerance &&
                    closestSurface <= tolerance && closestVector <= tolerance && (gap == 0 || closestDirection <= directionTol);
    }
    const auto& method = NarrowPhaseAlgorithm::between(ma.shape, mb.shape);
    const auto patch = method.collide(ma.shape, pa, mb.shape, pb, closest, margin, kContactDistance, kOnBoundary);
    const bool presence = (patch.count > 0) == (gap <= margin) && patch.count >= 0 && size_t(patch.count) == patch.points.size();
    bool witness = true;
    double deepest = na, worstSurface = na, worstVector = na, manifoldPlane = na, manifoldDirection = na;
    if (patch.count > 0) {
        deepest = std::numeric_limits<double>::infinity();
        worstSurface = worstVector = 0;
        manifoldPlane = planeGap(ma, pa, mb, pb, patch.normal);
        manifoldDirection = (patch.normal - n).length();
        // При g=0 нормаль в конусе угла может быть неединственна.
        witness = finite(patch.normal) && std::abs(patch.normal.length() - 1) <= directionTol &&
                  std::abs(manifoldPlane - gap) <= tolerance && (gap == 0 || manifoldDirection <= directionTol);
        for (const auto& p : patch.points) {
            witness &= finite(p.onA) && finite(p.onB) && std::isfinite(p.separation) && p.separation <= margin + tolerance;
            deepest = std::min(deepest, double(p.separation));
            worstSurface = std::max(worstSurface, surfaceError(ma, pa, mb, pb, p.onA, p.onB));
            worstVector = std::max(worstVector, double((p.onB - p.onA - p.separation * patch.normal).length()));
        }
        witness &= std::abs(deepest - gap) <= tolerance && worstSurface <= tolerance && worstVector <= tolerance;
    }
    const bool pass = oracle && closestOk && presence && witness;
    std::printf("%s,%s,%s,%d,%.9g,%.12g,%.12g,%d,%.12g,%.12g,%.12g,%.12g,%d,%.12g,%.12g,%.12g,%.12g,%.12g,"
                "%d,%d,%d,%d,%d\n", a.name, b.name, frame, swap, gap, lower, upper, int(resolved), distance,
                closestSurface, closestVector, closestDirection, patch.count, deepest, worstSurface, worstVector,
                manifoldPlane, manifoldDirection, int(oracle), int(closestOk), int(presence), int(witness), int(pass));
    return pass;
}
}

int main() {
    using namespace kp;
    const Sphere sphere(.2);
    const Capsule capsule(.17, .11);
    const Cylinder sharp(.23, .16), rounded(.23, .16, .025);
    const auto box = ConvexHull::box({.19, .23, .13});
    const Model ball{Family::ball, sphere, .2, 0, 0}, pill{Family::capsule, capsule, .11, .17, 0};
    // Номинально острый цилиндр SDK имеет постоянное скругление 32 мкм при этих размерах.
    const Model thin{Family::cylinder, sharp, .16, .23, .000032}, thick{Family::cylinder, rounded, .16, .23, .025};
    const Model brick{Family::box, box, 0, 0, 0, {.19, .23, .13}};
    const Vector3 dome = .6 * radial + .8 * up;
    std::vector<Feature> features{
        {"sphere", ball, up, ball.radius * up},
        {"capsule_dome", pill, dome, pill.halfHeight * up + pill.radius * dome},
        {"capsule_side", pill, radial, pill.radius * radial - .3 * pill.halfHeight * up},
        // Поперечная нормаль поддерживает целый отрезок: выбираем его конец на стыке.
        {"capsule_seam", pill, radial, pill.radius * radial + pill.halfHeight * up},
        {"hull_face", brick, up, {.02, .23, -.04}},
        {"hull_edge", brick, unit({2, 3, 0}), {.19, .23, .02}},
        {"hull_vertex", brick, unit({2, 3, 4}), brick.half}};
    cylinderFeatures(features, thin, {"sharp_cap", "sharp_side", "sharp_rim_capward", "sharp_rim_sideward"});
    cylinderFeatures(features, thick, {"rounded_cap", "rounded_side", "rounded_rim_capward", "rounded_rim_sideward"});
    const std::array<Transform, 2> frames{{{}, {{7.25, -3.75, 12.5}, Quaternion{.82, .19, -.31, .42}.unit()}}};
    std::puts("feature_a,feature_b,frame,swap,gap_m,oracle_plane_m,oracle_witness_distance_m,closest_resolved,"
              "closest_distance_m,closest_surface_error_m,closest_vector_error_m,closest_normal_error,point_count,"
              "min_contact_gap_m,max_contact_surface_error_m,max_contact_vector_error_m,reported_normal_plane_m,"
              "contact_normal_error,oracle_ok,closest_ok,presence_ok,witness_ok,pass");
    int rows = 0, failures = 0, expectedHits = 0, expectedMisses = 0;
    for (size_t i = 0; i < features.size(); ++i) for (size_t j = i; j < features.size(); ++j)
        for (size_t k = 0; k < frames.size(); ++k) for (double gap : {0., margin / 2, 20 * margin}) {
            const auto &a = features[i], &b = features[j];
            const auto pa = place(a, false, frames[k], gap), pb = place(b, true, frames[k], gap);
            const Vector3 n = frames[k].direction(axis);
            const char* frame = k ? "rotated_translated" : "identity";
            failures += !observe(a, pa, b, pb, n, frame, 0, gap);
            failures += !observe(b, pb, a, pa, -n, frame, 1, gap);
            rows += 2;
            if (gap <= margin) expectedHits += 2;
            else expectedMisses += 2;
        }
    std::fprintf(stderr, "features=%zu rows=%d expected_hit=%d expected_miss=%d failures=%d\n",
                 features.size(), rows, expectedHits, expectedMisses, failures);
    return features.size() == 15 && rows == 1440 && expectedHits == 960 && expectedMisses == 480 && !failures ? 0 : 1;
}
