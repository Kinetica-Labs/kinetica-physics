#include "../SupportGraph.h"
#include <dynamics/time_stepping/IslandNormals.h>

namespace kp {
namespace {

// u — скорость сближения со знаком вдоль нормали A → B, м/с. За h разрешено
// закрыть положительный зазор. Поправки мельче полосы контакта оставляем основному решателю.
void projectSupport(SoftContact& contact, size_t point, StepArray<SolverBody>& states, Real h) {
    const auto row = contact.normalRow(point);
    const auto& a = states[row.a];
    const auto& b = states[row.b];
    const Real u = row.normal.dot(b.v + b.w.cross(row.rB) - a.v - a.w.cross(row.rA));
    const Real target = -std::max(row.gap, Real(0)) / h;
    const Real excess = target - u;
    if (excess * h <= kContactDistance) return;
    const Real response = SolverBody::inverseMassAlong(a, b, row.rA, row.rB, row.normal);
    if (response > 0) contact.setNormalImpulse(states, point, row.impulse + excess / response); // Н·с, обеим сторонам
}
} // namespace

void IslandNormals::solveSupports(StepArray<SoftContact>& contacts, const std::vector<Body>& bodies,
                                 StepArray<SolverBody>& states, Vector3 gravity, Real h) const {
    if (!hasLayers_) return;
    const auto collect = [&](SupportGraph& graph) {
        for (size_t i = 0; i < contacts.size(); ++i) {
            if (methods_[i] != Method::Layered) continue;
            const SoftContact& c = contacts[i];
            for (size_t k = 0; k < c.pointCount(); ++k) {
                const auto row = c.normalRow(k);
                if (row.gap <= kContactDistance) {
                    graph.add(i, row.a, row.b, row.normal);
                    break;
                }
            }
        }
    };
    sweepSupports(bodies, gravity, collect, [&](size_t i, bool reverse) {
        SoftContact& c = contacts[i];
        const size_t count = c.pointCount();
        for (size_t k = 0; k < count; ++k) projectSupport(c, reverse ? count - 1 - k : k, states, h);
    });
}

} // namespace kp
