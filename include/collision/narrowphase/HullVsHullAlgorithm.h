#pragma once
// Контакт двух многогранников: отсечение грани одного по контуру грани другого.

#include <collision/narrowphase/NarrowPhaseAlgorithm.h>
#include <collision/shapes/ConvexHull.h>

namespace kp {

class HullVsHullAlgorithm final : public NarrowPhaseAlgorithm {
public:
    ContactManifold collide(const ConvexShape& a, const Transform& poseA, const ConvexShape& b, const Transform& poseB, const ClosestPoints& closest, Real margin,
                            Real touching, Real boundary) const override;
    // Только отсечение граней, без запасного контакта ближайшими точками.
    static ContactManifold clipFaces(const ConvexHull& a, const Transform& poseA, const ConvexHull& b, const Transform& poseB, Real margin, Real touching = 0, Real boundary = kOnBoundary);
};

} // namespace kp
