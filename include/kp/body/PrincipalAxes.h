#pragma once
// Главные оси и главные моменты симметричного тензора инерции.

#include <kp/math/Quaternion.h>

namespace kp {

// Главные оси симметричного тензора инерции (он при этом приводится к диагональному виду):
// поворот «главные оси → оси, в которых задан тензор» и моменты инерции вокруг главных осей.
struct PrincipalAxes {
    Quaternion orientation;
    Vector3 moments;

    static PrincipalAxes of(Real inertia[3][3]);
};

} // namespace kp
