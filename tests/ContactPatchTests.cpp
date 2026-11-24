#include "Check.h"

#include <collision/narrowphase/ContactPatch.h>

#include <cmath>

namespace {

void checkDiscBorders(kp::Real radius, kp::Real separation, kp::Real angle, int expected) {
    using namespace kp;
    Flat base;
    base.radius = radius;
    base.normal = {0, 0, 1};
    base.side = {1, 0, 0};
    Outline rim;
    rim.kind = Outline::Kind::Circle;
    rim.radius = .1;
    const Real offset = separation / std::sqrt(Real(2));
    rim.centre = {offset, offset, 0};
    rim.normal = {0, -std::sin(angle), std::cos(angle)};
    rim.points = Flat::quarterPoints(rim.centre, rim.normal, rim.radius, base.side);
    int borders = 0;
    for (const auto& point : ContactPatch::pointsOf(base, rim)) {
        if (point.id < 0x100 || point.id >= 0x200) continue;
        ++borders;
        CHECK_NEAR((point.core - rim.centre).dot(rim.normal), 0, kOnBoundary);
        CHECK_NEAR((point.core - rim.centre).length(), rim.radius, kOnBoundary);
        CHECK_NEAR(base.flatten(point.core - base.centre).length(), base.radius, kOnBoundary);
    }
    CHECK(borders == expected, "пересечений границ %d, ожидалось %d", borders, expected);
}

} // namespace

void contactPatchTests() {
    // Проекции центров смещены под 45° к линии пересечения плоскостей.
    checkDiscBorders(.1, .199, .001, 2);
    checkDiscBorders(.1, .201, .001, 0);
    // Радиус круга строго между полуосями концентрического эллипса.
    checkDiscBorders(.05 * (1 + std::cos(.2)), 0, .2, 4);
}
