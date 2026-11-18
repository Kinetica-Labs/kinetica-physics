#include <kp/collision/shapes/Capsule.h>

#include <kp/collision/shapes/Flat.h>
#include <kp/collision/shapes/Outline.h>
#include <kp/collision/shapes/Tube.h>

namespace kp {

Outline Capsule::outlineOver(const Transform& pose, const Flat&) const {
    Outline outline;
    outline.rounding = radius;
    const Vector3 axis = pose.direction({0, 1, 0});
    outline.kind = Outline::Kind::Segment;
    outline.points = {pose.position - halfLength * axis, pose.position + halfLength * axis};
    return outline;
}

std::optional<Tube> Capsule::tubeTowards(const Transform& pose, Vector3) const { return Tube{pose.position, pose.direction({0, 1, 0}), halfLength, radius}; }

} // namespace kp
