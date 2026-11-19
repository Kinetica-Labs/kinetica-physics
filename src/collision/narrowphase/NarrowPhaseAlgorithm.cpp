#include <collision/narrowphase/NarrowPhaseAlgorithm.h>

#include <collision/narrowphase/HullVsHullAlgorithm.h>
#include <collision/narrowphase/HullVsRoundAlgorithm.h>
#include <collision/narrowphase/RoundVsRoundAlgorithm.h>

namespace kp {

const NarrowPhaseAlgorithm& NarrowPhaseAlgorithm::between(const ConvexShape& a, const ConvexShape& b) {
    static const HullVsHullAlgorithm hulls;
    static const HullVsRoundAlgorithm mixed;
    static const RoundVsRoundAlgorithm rounds;
    const int hullCount = (a.hull() != nullptr) + (b.hull() != nullptr);
    if (hullCount == 2) return hulls;
    if (hullCount == 1) return mixed;
    return rounds;
}

} // namespace kp
