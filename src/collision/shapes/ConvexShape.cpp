#include <collision/shapes/ConvexShape.h>

#include <collision/shapes/Flat.h>
#include <collision/shapes/Outline.h>
#include <collision/shapes/Tube.h>

namespace kp {

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
