#pragma once

#include <kp/collision/shapes/ConvexShape.h>

namespace kp {

// Ядро капсулы: отрезок вдоль оси Y от −halfLength до +halfLength.
class Capsule final : public ConvexShape {
public:
    Capsule(Real halfLength_, Real r) : halfLength(halfLength_) { radius = r; }
    Vector3 support(Vector3 direction) const override { return {0, direction.y < 0 ? -halfLength : halfLength, 0}; }
    Real boundingRadius() const override { return halfLength + radius; }
    Outline outlineOver(const Transform& pose, const Flat& flat) const override;
    std::optional<Tube> tubeTowards(const Transform& pose, Vector3 towards) const override;
    Real halfLength;
};

} // namespace kp
