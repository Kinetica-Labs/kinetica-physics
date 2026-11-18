#pragma once
// Контакт двух круглых форм: площадка на торце цилиндра, общий участок двух трубок или ближайшие
// точки.

#include <kp/collision/narrowphase/NarrowPhaseAlgorithm.h>

namespace kp {

class RoundVsRoundAlgorithm final : public NarrowPhaseAlgorithm {
public:
    ContactManifold collide(const ConvexShape& a, const Transform& poseA, const ConvexShape& b, const Transform& poseB, const ClosestPoints& closest, Real margin,
                            Real touching, Real boundary) const override;
};

} // namespace kp
