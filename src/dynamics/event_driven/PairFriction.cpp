#include <math/StepMemory.h>
#include <dynamics/event_driven/PairFriction.h>

#include <algorithm>
#include <array>
#include <limits>

namespace kp {
namespace {

constexpr Real kIdle = ContactRows::kIdle;
constexpr Real kSliding = ContactRows::kSliding;

// Два единичных вектора, перпендикулярных n и друг другу. Выбор зависит только от n.
void tangents(Vector3 n, Vector3& t1, Vector3& t2) {
    const Vector3 axis = abs(n.x) < Real(0.57) ? Vector3{1, 0, 0} : Vector3{0, 1, 0};
    const Vector3 t = n.cross(axis);
    t1 = (1 / t.length()) * t;
    t2 = n.cross(t1);
}

} // namespace

// Обратная к симметричной положительной матрице 3 × 3 — через алгебраические дополнения.
PairFriction::Response PairFriction::inverted(const Response& m) {
    const Real c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1], c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2],
               c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    const Real determinant = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
    return {{{c00 / determinant, (m[0][2] * m[2][1] - m[0][1] * m[2][2]) / determinant, (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / determinant},
             {c01 / determinant, (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / determinant, (m[0][2] * m[1][0] - m[0][0] * m[1][2]) / determinant},
             {c02 / determinant, (m[0][1] * m[2][0] - m[0][0] * m[2][1]) / determinant, (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / determinant}}};
}

// Трение пары: центр точек касания, матрица отклика и скольжение до действия сил.
PairFriction::Pair PairFriction::frictionOf(const ContactPair& contact, size_t first, size_t last) const {
    Pair pair{};
    pair.first = first, pair.last = last, pair.a = contact.a, pair.b = contact.b;
    pair.normal = contact.manifold.normal;
    tangents(pair.normal, pair.tangent1, pair.tangent2);
    pair.friction = sqrt(state_.bodies[size_t(pair.a)].friction * state_.bodies[size_t(pair.b)].friction);
    pair.roundA = contact.manifold.roundA, pair.roundB = contact.manifold.roundB;
    // Центр — центр давления касающихся точек; если давление неизвестно — их среднее; если
    // касающихся точек нет — среднее всех точек пары.
    bool anyTouching = false;
    for (size_t i = first; i < last; ++i) anyTouching = anyTouching || state_.rows[i].touching;
    StepArray<Real> weight(last - first, 0);
    Real total = 0;
    for (size_t i = first; i < last; ++i) {
        if (!state_.rows[i].touching || !state_.pressure) continue;
        const auto known = state_.pressure->find(state_.rows[i].key);
        if (known != state_.pressure->end()) total += weight[i - first] = std::max(Real(0), known->second.normal);
    }
    if (total == 0)
        for (size_t i = first; i < last; ++i) total += weight[i - first] = state_.rows[i].touching == anyTouching;
    for (size_t i = first; i < last; ++i) pair.centre += (weight[i - first] / total) * state_.rows[i].point;
    for (size_t i = first; i < last; ++i) pair.reach += (weight[i - first] / total) * (state_.rows[i].point - pair.centre).length();

    const Response response = frictionResponse(pair, true, true);
    for (int k = 0; k < 3; ++k) pair.response[size_t(k)] = response[size_t(k)][size_t(k)];
    pair.inverse = inverted(response);
    pair.startSlip = slip(pair);
    pair.drift = {};
    return pair;
}

// Матрица отклика трения пары: K[i][j] — на сколько изменится скольжение i от единичного импульса
// j. Скольжение и импульс — три числа: вдоль двух касательных в центре и вокруг нормали. withA и
// withB — какие из двух тел подвижны: для прохода ударной волны нижнее считается неподвижным.
PairFriction::Response PairFriction::frictionResponse(const Pair& pair, bool withA, bool withB) const {
    Response k{};
    const Vector3 tangent[2] = {pair.tangent1, pair.tangent2};
    const struct {
        int index;
        Real sign;
        bool moving;
    } sides[2] = {{pair.a, -1, withA}, {pair.b, 1, withB}};
    for (const auto& side : sides) {
        const Body& body = state_.bodies[size_t(side.index)];
        if (!side.moving || body.isStatic()) continue;
        const Vector3 r = pair.centre - body.position;
        for (size_t j = 0; j < 3; ++j) {
            // Отклик тела на единичный импульс j: изменение скорости и угловой скорости.
            const Vector3 linear = j < 2 ? (side.sign * body.mass.inverseMass) * tangent[j] : Vector3{};
            const Vector3 angular = side.sign * inverseInertiaTimes(body, j < 2 ? r.cross(tangent[j]) : pair.normal);
            for (size_t i = 0; i < 2; ++i) k[i][j] += side.sign * tangent[i].dot(linear + angular.cross(r));
            k[2][j] += side.sign * pair.normal.dot(angular);
        }
    }
    if (pair.reach == 0) { // одна точка касания: вращению вокруг нормали трение не мешает
        for (size_t i = 0; i < 3; ++i) k[i][2] = k[2][i] = 0;
        k[2][2] = 1;
    }
    return k;
}

// Скольжение тела b по телу a в центре пары: вдоль двух касательных и вокруг нормали.
PairFriction::Slip PairFriction::slip(const Pair& pair) const {
    const Body& a = state_.bodies[size_t(pair.a)];
    const Body& b = state_.bodies[size_t(pair.b)];
    const Vector3 relative = pointVelocity(b, pair.centre) - pointVelocity(a, pair.centre);
    return {pair.tangent1.dot(relative), pair.tangent2.dot(relative),
            pair.reach > 0 ? pair.normal.dot(b.angularVelocity() - a.angularVelocity()) : Real(0)};
}

// Импульс трения пары: сила в центре и момент вокруг нормали, с противоположным знаком на два тела.
void PairFriction::applyFriction(const Pair& pair, const Slip& impulse, bool toA, bool toB) {
    const Vector3 force = impulse[0] * pair.tangent1 + impulse[1] * pair.tangent2;
    Body& a = state_.bodies[size_t(pair.a)];
    Body& b = state_.bodies[size_t(pair.b)];
    if (toB && !b.isStatic()) applyImpulse(b, force, pair.centre), b.angularMomentum += impulse[2] * pair.normal;
    if (toA && !a.isStatic()) applyImpulse(a, -force, pair.centre), a.angularMomentum += (-impulse[2]) * pair.normal;
}

// Импульс трения, который гасит скольжение пары, в пределах закона Кулона. response и inverse —
// матрица отклика и обратная к ней; возвращает новый накопленный импульс.
PairFriction::Slip PairFriction::frictionImpulse(const Pair& pair, const StepArray<Real>& goal, const Response& inverse) const {
    // Предел силы — μ Σ λn; предел момента — μ Σ λn rᵢ, где rᵢ — расстояние точки от центра.
    Real load = 0, leverLoad = 0;
    for (size_t i = pair.first; i < pair.last; ++i) {
        if (goal[i] == kIdle) continue;
        load += state_.sum[i];
        leverLoad += state_.sum[i] * (state_.rows[i].point - pair.centre).length();
    }
    const Slip now = slip(pair);
    Slip x = pair.impulse;
    for (size_t k = 0; k < 3; ++k)
        for (size_t j = 0; j < 3; ++j) x[k] -= inverse[k][j] * now[j];
    // На отрезке трение не толкает пару туда, куда она скользила в его начале. Работа постоянного
    // импульса равна x · (s₀ + s₁) / 2; импульс, гасящий конечное скольжение s₁, против него и
    // направлен, а без этого запрета он мог бы быть направлен вдоль s₀ — когда прочие силы сами
    // разворачивают скольжение, и трение «придерживает» тело, добавляя ему энергию.
    const Slip& start = pair.startSlip;
    const Real speed = start[0] * start[0] + start[1] * start[1] + pair.reach * pair.reach * start[2] * start[2];
    const Real power = x[0] * start[0] + x[1] * start[1] + x[2] * start[2];
    if (state_.holding && speed >= kSliding * kSliding && power > 0) {
        x[0] -= power * start[0] / speed;
        x[1] -= power * start[1] / speed;
        x[2] -= power * pair.reach * pair.reach * start[2] / speed;
    }
    const Real limit = pair.friction * load, size = sqrt(x[0] * x[0] + x[1] * x[1]);
    if (size > limit) x[0] *= limit / size, x[1] *= limit / size;
    x[2] = std::max(-pair.friction * leverLoad, std::min(pair.friction * leverLoad, x[2]));
    return x;
}

// Трение одной пары тел. Возвращает наибольшую поправку скорости.
Real PairFriction::solveFriction(Pair& pair, const StepArray<Real>& goal) {
    bool active = false;
    for (size_t i = pair.first; i < pair.last; ++i) active = active || goal[i] != kIdle;
    if (!active) return 0;
    const Slip x = frictionImpulse(pair, goal, pair.inverse);
    const Slip change = {x[0] - pair.impulse[0], x[1] - pair.impulse[1], x[2] - pair.impulse[2]};
    applyFriction(pair, change, true, true);
    pair.impulse = x;
    return std::max({abs(change[0]) * pair.response[0], abs(change[1]) * pair.response[1],
                     abs(change[2]) * pair.response[2] * pair.reach});
}

} // namespace kp
