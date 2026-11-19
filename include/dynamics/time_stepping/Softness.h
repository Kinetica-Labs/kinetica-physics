#pragma once
// Мягкая связь как пружина с демпфером, решённая неявно (Box3D, b3MakeSoft): поправка положения —
// biasRate · нарушение, доля импульса massScale, возврат накопленного impulseScale.

#include <math/math_functions.h>

namespace kp {

struct Softness {
    Real biasRate, massScale, impulseScale;

    static Softness make(Real hertz, Real damping, Real h) {
        const Real omega = 2 * kPi * hertz, a1 = 2 * damping + h * omega, a2 = h * omega * a1, a3 = 1 / (1 + a2);
        return {omega / a1, a2 * a3, a3};
    }
    static Softness rigid() { return {0, 1, 0}; }
};

} // namespace kp
