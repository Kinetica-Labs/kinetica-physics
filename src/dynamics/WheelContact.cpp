#include <dynamics/WheelContact.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace kp {
WheelContact::WheelContact(WheelSettings settings) : settings_(settings) {
    for (Real value : {settings.stiffness, settings.damping, settings.longitudinalDrag, settings.lateralDrag,
                       settings.longitudinalFriction, settings.lateralFriction})
        if (!(value >= 0) || !std::isfinite(value))
            throw std::invalid_argument("wheel coefficients must be finite and nonnegative");
}

WheelForces WheelContact::forces(Real compression, Real compressionRate, Real longitudinalSlip, Real lateralSlip) const {
    for (Real value : {compression, compressionRate, longitudinalSlip, lateralSlip})
        if (!std::isfinite(value)) throw std::invalid_argument("wheel contact state must be finite");
    if (compression <= 0) return {};
    const auto& s = settings_;
    const Real load = std::max(Real(0), s.stiffness * compression + s.damping * compressionRate);
    if (load == 0) return {};

    // При совместном скольжении (Fx/μxN)² + (Fy/μyN)² ≤ 1.
    const Real capX = s.longitudinalFriction * load, capY = s.lateralFriction * load;
    const Real fx = capX > 0 ? -s.longitudinalDrag * longitudinalSlip : Real(0);
    const Real fy = capY > 0 ? -s.lateralDrag * lateralSlip : Real(0);
    const Real x = capX > 0 ? fx / capX : Real(0), y = capY > 0 ? fy / capY : Real(0);
    const Real scale = std::max(Real(1), sqrt(x*x + y*y));
    return {load, fx / scale, fy / scale};
}
} // namespace kp
