#include <dynamics/time_stepping/TrackedShapes.h>

#include <collision/shapes/Sphere.h>

namespace kp {

void TrackedShapes::track(const Body& partA, const Body& partB) {
    if (dynamic_cast<const Sphere*>(partA.shape.get()) && dynamic_cast<const Sphere*>(partB.shape.get())) return;
    shapeA_ = partA.shape.get(), shapeB_ = partB.shape.get();
    fromA_ = {partA.pose().position - partA.position, partA.pose().orientation};
    fromB_ = {partB.pose().position - partB.position, partB.pose().orientation};
}

ClosestPoints TrackedShapes::closest(const BodyMotion& a, const BodyMotion& b, Vector3 apart) const {
    const Transform poseA{a.moved + a.turned.rotate(fromA_.position), (a.turned * fromA_.orientation).unit()};
    const Transform poseB{apart + b.moved + b.turned.rotate(fromB_.position), (b.turned * fromB_.orientation).unit()};
    return closestPoints(*shapeA_, poseA, *shapeB_, poseB);
}

} // namespace kp
