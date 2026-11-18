#include <kp/collision/shapes/Sphere.h>
#include <kp/collision/RayCast.h>

#include <kp/collision/narrowphase/Gjk.h>

namespace kp {
namespace {

constexpr Real kRayTouch = Real(1e-7); // м: ближе этого точка луча считается лежащей на поверхности
constexpr int kMaxRaySteps = 64;

// Шар с центром centre и радиусом radius лежит на пути луча не дальше reach.
bool nearRay(Vector3 origin, Vector3 unit, Real reach, Vector3 centre, Real radius) {
    const Vector3 to = centre - origin;
    const Real along = std::max(Real(0), std::min(reach, to.dot(unit)));
    return (to - along * unit).dot(to - along * unit) <= radius * radius;
}

// Луч и одна выпуклая форма. Точка луча идёт к форме шагами: ближайшая точка формы и нормаль в
// ней задают плоскость, за которой форма лежит целиком, и до этой плоскости на пути луча формы
// нет. Каждый шаг доводит точку до плоскости; у многогранника шагов не больше числа граней на
// пути, у гладкой формы расстояние убывает квадратично.
bool castOnShape(const ConvexShape& shape, const Transform& pose, Vector3 origin, Vector3 unit, Real reach, RayHit& hit) {
    const Sphere point(0);
    Real t = 0;
    for (int step = 0; step < kMaxRaySteps; ++step) {
        const ClosestPoints closest = closestPoints(point, {origin + t * unit, {}}, shape, pose);
        if (closest.coresOverlap || closest.distance <= kRayTouch) {
            hit.distance = t, hit.point = origin + t * unit, hit.normal = closest.coresOverlap ? -1 * unit : -1 * closest.normal;
            return true;
        }
        const Real approach = closest.normal.dot(unit);
        if (approach <= 0) return false; // луч идёт вдоль плоскости или от неё
        t += closest.distance / approach;
        if (t > reach) return false;
    }
    return false;
}

} // namespace

RayHit castRay(const std::vector<Body>& bodies, const Ray& ray) {
    const Vector3 origin = ray.origin, direction = ray.direction;
    Real reach = ray.reach;
    const Vector3 unit = (1 / direction.length()) * direction;
    RayHit nearest;
    for (size_t index = 0; index < bodies.size(); ++index) {
        const Body& body = bodies[index];
        if (!body.shape || !nearRay(origin, unit, reach, body.position, body.boundingRadius())) continue;
        for (int p = 0; p < body.partCount(); ++p) {
            const Body piece = body.part(p);
            const Transform pose = piece.pose();
            RayHit hit;
            if (!nearRay(origin, unit, reach, pose.position, piece.shape->boundingRadius()) || !castOnShape(*piece.shape, pose, origin, unit, reach, hit)) continue;
            hit.body = int(index), hit.part = p;
            nearest = hit, reach = hit.distance;
        }
    }
    return nearest;
}

} // namespace kp
