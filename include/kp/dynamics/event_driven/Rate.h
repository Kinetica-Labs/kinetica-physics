#pragma once
// Скорость изменения импульсов тела на отрезке шага: сила и момент, включая реакции опоры.

#include <kp/math/Vector3.h>

namespace kp {

struct Rate {
    Vector3 force, torque;
};

} // namespace kp
