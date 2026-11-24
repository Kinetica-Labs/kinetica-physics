#pragma once
// Сравнение наборов отметок без зависимости от порядка обхода площадок.
#include <debug/DebugGeometry.h>
#include <algorithm>

namespace overlay_test {
using namespace kp;
constexpr double visible = 0.001; // м, для каждого из двух концов направленной нормали
inline bool near(Vector3 a, Vector3 b, double tolerance) {
    if (tolerance == 0) return a.x == b.x && a.y == b.y && a.z == b.z;
    return double((a - b).lengthSquare()) <= tolerance * tolerance;
}
inline bool close(const DebugPoint& a, const DebugPoint& b, double tol) {
    return a.kind == b.kind && near(a.position, b.position, tol);
}
inline bool close(const DebugLine& a, const DebugLine& b, double tol) {
    return a.kind == b.kind && near(a.from, b.from, tol) && near(a.to, b.to, tol);
}
template <class Items>
bool sameOrder(const Items& a, const Items& b) {
    return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(),
        [](const auto& p, const auto& q) { return close(p, q, 0); });
}
inline bool exact(const DebugGeometry& a, const DebugGeometry& b) {
    return sameOrder(a.points, b.points) && sameOrder(a.lines, b.lines);
}
template <class Items>
bool visiblyDifferent(const Items& a, const Items& b) {
    if (sameOrder(a, b)) return false;
    const auto unmatched = [](const Items& from, const Items& to) {
        for (const auto& p : from)
            if (std::none_of(to.begin(), to.end(), [&](const auto& q) { return close(p, q, visible); })) return true;
        return false;
    };
    return unmatched(a, b) || unmatched(b, a);
}

} // namespace overlay_test
