#include <kp/joints/FixedJoint.h>

namespace kp {

void FixedJoint::build(const Motion& m, std::vector<Bond>& bonds) const {
    for (int k = 0; k < 3; ++k) lockTravel(m, k, bonds);
    lockAllTurns(m, bonds);
}

} // namespace kp
