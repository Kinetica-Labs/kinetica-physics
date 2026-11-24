#pragma once
#include <dynamics/time_stepping/DirectJoints.h>
#include <dynamics/time_stepping/SoftContact.h>
#include <array>

namespace kp {
class CoupledFriction {
public:
    CoupledFriction(SoftContact& first, SoftContact& second) : pair_{&first, &second} {}
    bool solve(StepArray<SolverBody>& states, const DirectJoints& guides, Real h, Real rollingArm);

private:
    std::array<Bond, 4> tangentRows() const;
    std::array<SoftContact*, 2> pair_;
};
}
