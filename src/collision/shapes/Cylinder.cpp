#include <kp/collision/shapes/Cylinder.h>

#include <kp/collision/shapes/Flat.h>
#include <kp/collision/shapes/Outline.h>
#include <kp/collision/shapes/Tube.h>

#include <cmath>
#include <vector>

namespace kp {
namespace {

constexpr Real kHalfRightAngle = Real(0.70710678118654752); // cos 45°: цилиндр обращён торцом или боком
// Цилиндр обращён к другому телу боком, если нормаль касания перпендикулярна его оси с такой
// точностью. Между боком и торцом лежит кромка: там цилиндр не трубка и не круг, и контакт один.
constexpr Real kSideFacing = Real(0.05);

} // namespace

std::optional<Flat> Cylinder::flatTowards(const Transform& pose, Vector3 towards) const {
    const Vector3 axis = pose.direction({0, 1, 0});
    if (abs(axis.dot(towards)) < kHalfRightAngle) return std::nullopt;
    Flat flat;
    flat.normal = (axis.dot(towards) > 0 ? Real(1) : Real(-1)) * axis;
    flat.centre = flat.origin = pose.position + coreHalfHeight * flat.normal;
    flat.side = pose.direction({1, 0, 0});
    flat.radius = coreRadius, flat.rounding = radius;
    return flat;
}

Outline Cylinder::outlineOver(const Transform& pose, const Flat& flat) const {
    Outline outline;
    outline.rounding = radius;
    const Vector3 axis = pose.direction({0, 1, 0});
    const Real along = axis.dot(flat.normal);
    if (abs(along) < kHalfRightAngle) {
        // На боку: нижняя образующая ядра — отрезок вдоль оси, смещённый к плоскости на радиус ядра.
        const Vector3 offset = coreRadius * Flat::inPlane(-1 * flat.normal, axis, {});
        outline.kind = Outline::Kind::Segment;
        outline.points = {pose.position - coreHalfHeight * axis + offset, pose.position + coreHalfHeight * axis + offset};
    } else {
        // На торце: окружность ядра. Первая точка — самая низкая над плоскостью; если торец ей
        // параллелен, отсчёт идёт от центра круга под ним или от направления, связанного с телом.
        outline.kind = Outline::Kind::Circle;
        outline.normal = (along > 0 ? Real(-1) : Real(1)) * axis;
        outline.centre = pose.position + coreHalfHeight * outline.normal;
        outline.radius = coreRadius;
        const Vector3 fixed = flat.disc() ? Flat::inPlane(flat.centre - outline.centre, outline.normal, pose.direction({1, 0, 0})) : pose.direction({1, 0, 0});
        outline.points = Flat::quarterPoints(outline.centre, outline.normal, outline.radius, Flat::inPlane(-1 * flat.normal, outline.normal, fixed));
    }
    return outline;
}

Real Cylinder::rollingRadius(const Transform& pose, Vector3 towards) const {
    return abs(pose.direction({0, 1, 0}).dot(towards)) < kHalfRightAngle ? coreRadius + radius : radius;
}

std::optional<Tube> Cylinder::tubeTowards(const Transform& pose, Vector3 towards) const {
    const Vector3 axis = pose.direction({0, 1, 0});
    if (abs(axis.dot(towards)) < kSideFacing) return Tube{pose.position, axis, coreHalfHeight, coreRadius + radius};
    return std::nullopt;
}

} // namespace kp
