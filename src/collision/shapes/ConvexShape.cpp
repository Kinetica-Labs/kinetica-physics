#include <collision/shapes/ConvexShape.h>

#include <collision/shapes/Flat.h>
#include <collision/shapes/Outline.h>
#include <collision/shapes/Tube.h>
#include <algorithm>

namespace kp {

Vector3 ConvexShape::coreExtents() const {
    const auto along = [&](Vector3 axis) {
        return std::max(axis.dot(support(axis)), -axis.dot(support(-1 * axis)));
    };
    return {along({1, 0, 0}), along({0, 1, 0}), along({0, 0, 1})};
}

std::optional<Flat> ConvexShape::flatTowards(const Transform&, Vector3) const { return std::nullopt; }

Outline ConvexShape::outlineOver(const Transform& pose, const Flat&) const {
    Outline outline;
    outline.rounding = radius;
    outline.points = {pose.position};
    return outline;
}

Real ConvexShape::rollingRadius(const Transform&, Vector3) const { return radius; }

std::optional<Tube> ConvexShape::tubeTowards(const Transform&, Vector3) const { return std::nullopt; }

} // namespace kp
