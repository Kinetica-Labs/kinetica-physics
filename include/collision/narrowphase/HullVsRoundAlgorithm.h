#pragma once
// Контакт многогранника с шаром, капсулой или цилиндром: площадка на грани или ближайшие точки.

#include <collision/narrowphase/NarrowPhaseAlgorithm.h>
#include <collision/shapes/ConvexHull.h>

namespace kp {

class HullVsRoundAlgorithm final : public NarrowPhaseAlgorithm {
public:
    ContactManifold collide(const ConvexShape& a, const Transform& poseA, const ConvexShape& b, const Transform& poseB, const ClosestPoints& closest, Real margin,
                            Real touching, Real boundary) const override;

private:
    static ContactManifold against(const ConvexHull& hull, const Transform& hullPose, const ConvexShape& round, const Transform& roundPose, const ClosestPoints& closest,
                                   Real margin, Real touching);
};

} // namespace kp
