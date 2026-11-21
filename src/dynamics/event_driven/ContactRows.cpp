#include <dynamics/event_driven/ContactRows.h>

#include <algorithm>
#include <array>
#include <limits>

namespace kp {
namespace {

// Ускорение точки касания тела, которое возникает без всяких сил — только оттого, что тело
// вращается. Точка, движущаяся вместе с телом, идёт по дуге: ω × (ω × r) — центростремительное
// ускорение; у тела с неравными моментами инерции меняется сама угловая скорость,
// α = I⁻¹ (L × ω), и добавляет α × r. Силы контакта это ускорение не создают, но обязаны его
// учитывать: тело, которое опрокидывается через точку опоры, без этого члена за шаг уводит её с
// места, а сила опоры и трение совершают работу на этом сдвиге. В уравнениях движения в
// обобщённых координатах это слагаемое J̇ v.
//
// У круглого тела точка касания перекатывается по поверхности, а вместе с телом движется точка
// ядра moving — центр шара, точка оси цилиндра: по дуге идёт она, а не точка касания contact.
Vector3 geometricAcceleration(const Body& body, Vector3 moving, Vector3 contact) {
    if (body.isStatic() && !isDriven(body)) return {}; // у движимого тела ω задана и постоянна: остаётся ω × (ω × r)
    const Vector3 w = body.angularVelocity();
    const Vector3 alpha = inverseInertiaTimes(body, body.angularMomentum.cross(w));
    return w.cross(w.cross(moving - body.position)) + alpha.cross(contact - body.position);
}

} // namespace

// Обратная эффективная масса вдоль направления d в точке с плечами rA, rB:
// d · K d, K = (1/mA + 1/mB) E − [rA]ₓ IA⁻¹ [rA]ₓ − [rB]ₓ IB⁻¹ [rB]ₓ.
Real ContactRows::effectiveMass(const Body& a, const Body& b, Vector3 point, Vector3 d) {
    const Vector3 ra = point - a.position, rb = point - b.position;
    const Real k = a.mass.inverseMass + b.mass.inverseMass + d.dot(inverseInertiaTimes(a, ra.cross(d)).cross(ra)) +
                   d.dot(inverseInertiaTimes(b, rb.cross(d)).cross(rb));
    return 1 / k;
}

// Ускорения точек касания без сил (geometricAcceleration) по скоростям тел moving — тех же тел в
// тех же позах. При создании решателя это сами тела; World уточняет их по средним за отрезок
// импульсам: угловая скорость тела, которое только что опёрлось, за шаг меняется сильно, и
// ускорение по скорости начала отрезка заметно ошибается.
Real ContactRows::setDrift(const std::vector<Body>& moving) {
    Real largest = 0;
    for (Row& row : rows) {
        const Body& a = moving[size_t(row.key.a)];
        const Body& b = moving[size_t(row.key.b)];
        // Нормаль поворачивается вместе с телом, которому принадлежит грань: вторая производная
        // зазора n · d равна n · d̈ + 2 ṅ · ḋ.
        const Vector3 turning = (row.faceOnB ? b : a).angularVelocity().cross(row.normal);
        const Vector3 relative = pointVelocity(b, row.point) - pointVelocity(a, row.point);
        const Vector3 coreA = row.point - row.roundA * row.normal, coreB = row.point + row.roundB * row.normal;
        row.drift = row.normal.dot(geometricAcceleration(b, coreB, row.point) - geometricAcceleration(a, coreA, row.point)) + 2 * turning.dot(relative);
        largest = std::max(largest, abs(row.drift));
    }
    for (Pair& pair : pairs) {
        const Body& a = moving[size_t(pair.a)];
        const Body& b = moving[size_t(pair.b)];
        const Vector3 sliding = geometricAcceleration(b, pair.centre + pair.roundB * pair.normal, pair.centre) -
                             geometricAcceleration(a, pair.centre - pair.roundA * pair.normal, pair.centre);
        const Vector3 spinning = inverseInertiaTimes(b, b.angularMomentum.cross(b.angularVelocity())) - inverseInertiaTimes(a, a.angularMomentum.cross(a.angularVelocity()));
        pair.drift = {pair.tangent1.dot(sliding), pair.tangent2.dot(sliding), pair.reach > 0 ? pair.normal.dot(spinning) : Real(0)};
        for (Real d : pair.drift) largest = std::max(largest, abs(d));
    }
    if (joints) {
        size_t k = 0;
        for (const Joint& joint : *joints)
            for (const Bond& bond : joint.bonds(bodies, moving)) largest = std::max(largest, abs(bonds[k++].drift = bond.drift));
    }
    return largest;
}

// Скорость тела b относительно тела a в точке контакта вдоль направления; отрицательная вдоль
// нормали — тела сближаются.
Real ContactRows::relativeSpeed(const Row& row, Vector3 direction) const {
    return direction.dot(pointVelocity(bodies[size_t(row.key.b)], row.point) - pointVelocity(bodies[size_t(row.key.a)], row.point));
}

// Импульс вдоль линии: телу b — сила linear и момент angularB, телу a — противоположные.
void ContactRows::applyLine(const Line& line, Real impulse) {
    Body& a = bodies[size_t(line.a)];
    Body& b = bodies[size_t(line.b)];
    if (!b.isStatic()) b.linearMomentum += impulse * line.linear, b.angularMomentum += impulse * line.angularB;
    if (!a.isStatic()) a.linearMomentum += -impulse * line.linear, a.angularMomentum += -impulse * line.angularA;
}

void ContactRows::apply(const Row& row, Vector3 impulse) {
    applyImpulse(bodies[size_t(row.key.b)], impulse, row.point);
    applyImpulse(bodies[size_t(row.key.a)], -impulse, row.point);
}

Real ContactRows::kineticEnergy() const {
    Real total = 0;
    for (const Body& body : bodies)
        if (!body.isStatic()) total += kineticEnergyOf(body);
    return total;
}

// Линия нормали в точке контакта.
ContactRows::Line ContactRows::normalLine(const Row& row) const {
    return {row.key.a, row.key.b, row.normal, (row.point - bodies[size_t(row.key.a)].position).cross(row.normal),
            (row.point - bodies[size_t(row.key.b)].position).cross(row.normal)};
}

// Линия трения пары: k = 0, 1 — вдоль касательных в центре, k = 2 — вращение вокруг нормали.
ContactRows::Line ContactRows::frictionLine(const Pair& pair, int k) const {
    if (k == 2) return {pair.a, pair.b, {}, pair.normal, pair.normal};
    const Vector3 d = k == 0 ? pair.tangent1 : pair.tangent2;
    return {pair.a, pair.b, d, (pair.centre - bodies[size_t(pair.a)].position).cross(d), (pair.centre - bodies[size_t(pair.b)].position).cross(d)};
}

Real ContactRows::speed(const Line& line) const {
    const Body& a = bodies[size_t(line.a)];
    const Body& b = bodies[size_t(line.b)];
    return line.linear.dot(b.linearVelocity() - a.linearVelocity()) + line.angularB.dot(b.angularVelocity()) - line.angularA.dot(a.angularVelocity());
}

Real ContactRows::driveCap(size_t bond) const {
    const Real strength = bonds[bond].strength;
    return strength == std::numeric_limits<Real>::infinity() ? strength : strength * holdTime;
}

} // namespace kp
