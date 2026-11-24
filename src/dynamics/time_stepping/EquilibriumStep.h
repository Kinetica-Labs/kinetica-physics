#pragma once

#include <dynamics/time_stepping/SoftStep.h>
#include <dynamics/time_stepping/SolverBody.h>
#include <math/StepMemory.h>

namespace kp {

// Пробный подшаг для покоящихся многогранников. При false тела и память обеих групп связей прежние.
bool advanceIfBalanced(const SoftStep& step, std::vector<Body>& bodies, const std::vector<ContactPair>& pairs,
                       const JointSet& joints, Vector3 gravity, Real h, ContactForces& forces,
                       const std::vector<std::vector<Real>>& jointMemory, const StepArray<SolverBody>& initial);

} // namespace kp
