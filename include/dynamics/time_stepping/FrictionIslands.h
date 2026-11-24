#pragma once
#include <dynamics/time_stepping/ContactLevels.h>
#include <dynamics/time_stepping/DirectJoints.h>
#include <dynamics/time_stepping/SoftJoint.h>

namespace kp {
// Два контакта с запертыми связями решаются вместе. Выбор зависит только от связей данного острова.
void solveFrictionIslands(StepArray<SoftContact>& contacts, StepArray<SolverBody>& states, const DirectJoints& locked,
                          const StepArray<SoftJoint>& soft, ContactLevels& levels, Real h, Real rollingArm);
}
