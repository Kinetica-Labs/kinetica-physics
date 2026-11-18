#include <kp/math/Quaternion.h>

namespace kp {

Quaternion Quaternion::fromRotationVector(Vector3 theta) {
    const Real angle = theta.length();
    const SinCos half = sinCos(angle / 2);
    const Real scale = angle < Real(1e-4) ? Real(0.5) - angle * angle / 48 : half.sin / angle;
    return {half.cos, scale * theta.x, scale * theta.y, scale * theta.z};
}

} // namespace kp
