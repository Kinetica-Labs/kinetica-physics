#include <kp/joints/JointSet.h>

#include <algorithm>
#include <cmath>

namespace kp {

JointSet::JointSet(const JointSet& other) {
    for (const Joint& joint : other) joints_.push_back(joint.clone());
}

Real JointSet::largestError(const std::vector<Body>& bodies) const {
    Real largest = 0;
    for (const Joint& joint : *this)
        for (const Bond& bond : joint.bonds(bodies, bodies)) largest = std::max(largest, bond.oneSided ? -bond.gap : abs(bond.error));
    return largest;
}

} // namespace kp
