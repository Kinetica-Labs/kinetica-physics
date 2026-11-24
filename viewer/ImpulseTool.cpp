#include "ImpulseTool.h"

#include <math/math_functions.h>

using namespace kp;

namespace {

constexpr int kArc = 48; // звеньев в окружности
constexpr Real kTurn = Real(6.283185307179586);

// Окружность радиуса radius в плоскости единичных u и v.
void circle(DebugGeometry& geometry, Vector3 center, Vector3 u, Vector3 v, Real radius) {
    for (int k = 0; k < kArc; ++k) {
        const SinCos from = sinCos(kTurn * Real(k) / kArc), to = sinCos(kTurn * Real(k + 1) / kArc);
        geometry.lines.push_back({center + radius * (from.cos * u + from.sin * v), center + radius * (to.cos * u + to.sin * v), DebugKind::ClosestPoints});
    }
}

// Стрелка из from длиной length вдоль единичного along, с наконечником из четырёх усов.
void arrow(DebugGeometry& geometry, Vector3 from, Vector3 along, Real length) {
    const Vector3 tip = from + length * along, side = kp::abs(along.y) < Real(0.9) ? along.cross({0, 1, 0}) : along.cross({1, 0, 0});
    const Vector3 u = (1 / side.length()) * side, v = along.cross(u);
    geometry.lines.push_back({from, tip, DebugKind::ClosestPoints});
    for (Vector3 barb : {u, -1 * u, v, -1 * v}) geometry.lines.push_back({tip, tip - (Real(0.2) * length) * along + (Real(0.08) * length) * barb, DebugKind::ClosestPoints});
}

} // namespace

Vector3 ImpulseTool::direction() const {
    const SinCos turn = sinCos(Real(yaw)), rise = sinCos(Real(pitch));
    return {rise.cos * turn.cos, rise.sin, rise.cos * turn.sin};
}

int ImpulseTool::fire(World& world, Real share) const {
    if (radial) return applyRadialImpulse(world, center, Real(radius), share * Real(impulse), falloff);
    return applyDirectedImpulse(world, center, Real(radius), (share * Real(impulse)) * direction(), falloff);
}

void ImpulseTool::draw(DebugGeometry& geometry) const {
    const Real reach = Real(radius);
    circle(geometry, center, {1, 0, 0}, {0, 1, 0}, reach), circle(geometry, center, {0, 1, 0}, {0, 0, 1}, reach), circle(geometry, center, {1, 0, 0}, {0, 0, 1}, reach);
    geometry.points.push_back({center, DebugKind::ClosestPoints});
    if (!radial) return arrow(geometry, center, direction(), reach);
    for (Vector3 axis : {Vector3{1, 0, 0}, Vector3{0, 1, 0}, Vector3{0, 0, 1}})
        arrow(geometry, center + (Real(0.3) * reach) * axis, axis, Real(0.4) * reach), arrow(geometry, center - (Real(0.3) * reach) * axis, -1 * axis, Real(0.4) * reach);
}
