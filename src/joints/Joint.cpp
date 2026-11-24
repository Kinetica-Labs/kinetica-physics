#include <joints/Joint.h>

namespace kp {
namespace {

Vector3 basis(int k) { return {Real(k == 0), Real(k == 1), Real(k == 2)}; }

// Угловое ускорение свободного тела: у тела с неравными моментами инерции угловая скорость
// меняется и без сил, α = I⁻¹ (L × ω).
Vector3 freeSpinUp(const Body& body) {
    return body.isStatic() ? Vector3{} : inverseInertiaTimes(body, body.angularMomentum.cross(body.angularVelocity()));
}

} // namespace

Quaternion Joint::turningXTo(Vector3 to) {
    const Real c = to.x; // косинус угла между x и to
    if (c < Real(-0.999999)) return {0, 0, 0, 1}; // пол-оборота вокруг z
    return Quaternion{1 + c, 0, -to.z, to.y}.unit();   // вектор кватерниона — x × to
}

Joint::Joint(int a_, const Body& bodyA, int b_, const Body& bodyB, Vector3 anchor, Vector3 axis) : a(a_), b(b_) {
    anchorA = bodyA.pose().localPoint(anchor), anchorB = bodyB.pose().localPoint(anchor);
    const Quaternion frame = turningXTo((1 / axis.length()) * axis);
    frameA = bodyA.orientation.conjugate() * frame, frameB = bodyB.orientation.conjugate() * frame;
}

std::vector<Bond> Joint::bonds(const std::vector<Body>& bodies, const std::vector<Body>& moving) const {
    std::vector<Bond> result;
    writeBonds(bodies, moving, result);
    return result;
}

void Joint::writeBonds(const std::vector<Body>& bodies, const std::vector<Body>& moving, std::vector<Bond>& result) const {
    result.clear();
    const Body &bodyA = bodies[size_t(a)], &bodyB = bodies[size_t(b)];
    if (bodyA.isStatic() && bodyB.isStatic()) return;
    const Body &ma = moving[size_t(a)], &mb = moving[size_t(b)];
    Motion m;
    m.bodyA = &bodyA, m.bodyB = &bodyB;
    m.frameA = bodyA.orientation * frameA, m.frameB = bodyB.orientation * frameB;
    m.wa = ma.angularVelocity(), m.wb = mb.angularVelocity(), m.spinUpA = freeSpinUp(ma), m.spinUpB = freeSpinUp(mb);
    // Перемещения. Нарушение — C = e · (p_b − p_a), где p — точки сочленения тел, e — ось тела a.
    // Его производная — разность скоростей точек двух тел, взятых в одной точке p_b, вдоль e:
    // слагаемое от поворота самой оси e как раз и переносит плечо тела a из p_a в p_b.
    m.pa = bodyA.pose().point(anchorA), m.pb = bodyB.pose().point(anchorB);
    m.armB = m.pb - bodyB.position, m.armA = m.pb - bodyA.position;
    m.pointB = mb.linearVelocity() + m.wb.cross(m.armB), m.pointA = ma.linearVelocity() + m.wa.cross(m.armA);
    // Ускорение точки p_b относительно тела a без сил: точка тела b идёт по дуге, тело a под ней
    // поворачивается.
    m.apart = m.wb.cross(m.wb.cross(m.armB)) + m.spinUpB.cross(m.armB) - m.spinUpA.cross(m.armA) - m.wa.cross(m.pointB - ma.linearVelocity());
    build(m, result);
}

Bond Joint::travelLine(const Motion& m, int axis) const { return travelLine(m, m.frameA.rotate(basis(axis))); }

Bond Joint::travelLine(const Motion& m, Vector3 e) const {
    return {a, b, e, m.armA.cross(e), m.armB.cross(e), e.dot(m.pb - m.pa), e.dot(m.apart) + m.wa.cross(e).dot(m.pointB - m.pointA)};
}

Bond Joint::turnLine(const Motion& m, int axis) const {
    const Vector3 e = m.frameA.rotate(basis(axis));
    return {a, b, {}, e, e, 0, e.dot(m.spinUpB - m.spinUpA) + m.wa.cross(e).dot(m.wb - m.wa)};
}

void Joint::lockTravel(const Motion& m, int axis, std::vector<Bond>& bonds) const { bonds.push_back(travelLine(m, axis)); }

// Все повороты заперты. Нарушение — поворот осей тела b относительно осей тела a; для малого
// поворота его вектор равен удвоенной векторной части кватерниона.
void Joint::lockAllTurns(const Motion& m, std::vector<Bond>& bonds) const {
    const Quaternion apartTurn = m.frameA.conjugate() * m.frameB;
    const Real sign = apartTurn.w < 0 ? Real(-2) : Real(2);
    const Vector3 turn = sign * Vector3{apartTurn.x, apartTurn.y, apartTurn.z};
    for (int k = 0; k < 3; ++k) {
        Bond bond = turnLine(m, k);
        bond.error = k == 0 ? turn.x : k == 1 ? turn.y : turn.z;
        bonds.push_back(bond);
    }
}

// Главная ось тела b обязана оставаться перпендикулярной двум другим осям тела a:
// C = x_b · u, dC/dt = (ω_b − ω_a) · (x_b × u).
void Joint::lockTurnsAcrossAxis(const Motion& m, std::vector<Bond>& bonds) const {
    const Vector3 axisB = m.frameB.rotate({1, 0, 0});
    for (int k = 1; k < 3; ++k) {
        const Vector3 u = m.frameA.rotate(basis(k)), n = axisB.cross(u);
        const Vector3 turning = m.wb.cross(axisB).cross(u) + axisB.cross(m.wa.cross(u)); // dn/dt
        bonds.push_back({a, b, {}, n, n, axisB.dot(u), (m.spinUpB - m.spinUpA).dot(n) + (m.wb - m.wa).dot(turning)});
    }
}

} // namespace kp
