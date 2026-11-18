#include <kp/joints/CylindricalJoint.h>

namespace kp {

void CylindricalJoint::build(const Motion& m, std::vector<Bond>& bonds) const {
    for (int k = 1; k < 3; ++k) lockTravel(m, k, bonds);
    lockTurnsAcrossAxis(m, bonds);
}

} // namespace kp
