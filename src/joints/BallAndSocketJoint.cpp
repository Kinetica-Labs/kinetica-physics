#include <joints/BallAndSocketJoint.h>

namespace kp {

void BallAndSocketJoint::build(const Motion& m, std::vector<Bond>& bonds) const {
    for (int k = 0; k < 3; ++k) lockTravel(m, k, bonds);
}

} // namespace kp
