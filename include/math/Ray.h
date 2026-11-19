#pragma once
// Луч: начало, направление и длина. Направление не обязано быть единичным; длина — в метрах.

#include <math/Vector3.h>

#include <limits>

namespace kp {

struct Ray {
    Vector3 origin, direction;
    Real reach = std::numeric_limits<Real>::infinity();
};

} // namespace kp
