#pragma once

#include <collision/shapes/ConvexShape.h>

namespace kp {

// Ядро шара.
class Sphere final : public ConvexShape {
public:
    explicit Sphere(Real r) { radius = r; }
    Vector3 support(Vector3) const override { return {}; }
    Real boundingRadius() const override { return radius; }
};

} // namespace kp
