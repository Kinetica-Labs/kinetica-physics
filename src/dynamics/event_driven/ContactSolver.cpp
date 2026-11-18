#include <kp/dynamics/event_driven/ContactSolver.h>

#include <kp/dynamics/event_driven/ActiveSet.h>

#include <algorithm>
#include <array>
#include <limits>

namespace kp {
namespace {

constexpr int kMaxIterations = 200;    // чередований «нормали — трение»
constexpr int kMaxDriveFlips = 4;      // сколько раз привод может упереться в предел силы и отпуститься
// Медленнее пара считается не скользящей, м/с: для неё сначала пробуется трение покоя. Поднять
// порог до уровня округления больших задач (10⁻⁷…10⁻⁶ м/с) нельзя: уже при 10⁻⁷ покоящаяся
// пирамида из 36 кубов начинает расползаться. По той же причине нельзя и не замечать остановки
// медленного скольжения: пара, ставшая скользящей, возвращается к трению покоя только событием
// остановки.
constexpr Real kSliding = Real(1e-9);
constexpr Real kHalted = Real(0.05);   // скольжение упало до этой доли начального — пара остановилась, см. SlidingStop
constexpr Real kIdle = -std::numeric_limits<Real>::infinity(); // цель точки, не участвующей в фазе

// Обратная эффективная масса вдоль направления d в точке с плечами rA, rB:
// d · K d, K = (1/mA + 1/mB) E − [rA]ₓ IA⁻¹ [rA]ₓ − [rB]ₓ IB⁻¹ [rB]ₓ.
Real effectiveMass(const Body& a, const Body& b, Vector3 point, Vector3 d) {
    const Vector3 ra = point - a.position, rb = point - b.position;
    const Real k = a.mass.inverseMass + b.mass.inverseMass + d.dot(a.inverseInertiaTimes(ra.cross(d)).cross(ra)) +
                   d.dot(b.inverseInertiaTimes(rb.cross(d)).cross(rb));
    return 1 / k;
}

// Два единичных вектора, перпендикулярных n и друг другу. Выбор зависит только от n.
void tangents(Vector3 n, Vector3& t1, Vector3& t2) {
    const Vector3 axis = abs(n.x) < Real(0.57) ? Vector3{1, 0, 0} : Vector3{0, 1, 0};
    const Vector3 t = n.cross(axis);
    t1 = (1 / t.length()) * t;
    t2 = n.cross(t1);
}

// Обратная к симметричной положительной матрице 3 × 3 — через алгебраические дополнения.
std::array<std::array<Real, 3>, 3> inverted(const std::array<std::array<Real, 3>, 3>& m) {
    const Real c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1], c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2],
               c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    const Real determinant = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
    return {{{c00 / determinant, (m[0][2] * m[2][1] - m[0][1] * m[2][2]) / determinant, (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / determinant},
             {c01 / determinant, (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / determinant, (m[0][2] * m[1][0] - m[0][0] * m[1][2]) / determinant},
             {c02 / determinant, (m[0][1] * m[2][0] - m[0][0] * m[2][1]) / determinant, (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / determinant}}};
}

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
    if (body.isStatic()) return {};
    const Vector3 w = body.angularVelocity();
    const Vector3 alpha = body.inverseInertiaTimes(body.angularMomentum.cross(w));
    return w.cross(w.cross(moving - body.position)) + alpha.cross(contact - body.position);
}

} // namespace

ContactSolver::ContactSolver(std::vector<Body>& bodies, const std::vector<ContactPair>& pairs, Vector3 up, Real restBounceSpeed, const ContactForces* pressure,
                             const JointSet* joints)
    : bodies_(bodies), pressure_(pressure), up_(up), restBounceSpeed_(restBounceSpeed), joints_(joints) {
    // Пары с точками контакта — по высоте самой низкой точки, снизу вверх; при равенстве — по
    // номерам тел, чтобы порядок зависел только от самих чисел.
    struct Ordered {
        Real height;
        const ContactPair* pair;
    };
    std::vector<Ordered> ordered;
    for (const ContactPair& pair : pairs) {
        if (pair.manifold.count == 0) continue;
        Real lowest = std::numeric_limits<Real>::infinity();
        for (int i = 0; i < pair.manifold.count; ++i) lowest = std::min(lowest, up.dot(pair.manifold.points[i].onA));
        ordered.push_back({lowest, &pair});
    }
    std::stable_sort(ordered.begin(), ordered.end(), [](const Ordered& x, const Ordered& y) { return x.height < y.height; });

    for (const Ordered& entry : ordered) {
        const ContactPair& pair = *entry.pair;
        const Body& a = bodies_[size_t(pair.a)];
        const Body& b = bodies_[size_t(pair.b)];
        const size_t first = rows_.size();
        for (int i = 0; i < pair.manifold.count; ++i) {
            const ContactPoint& p = pair.manifold.points[i];
            Row row;
            row.key = {pair.a, pair.b, p.id, pair.partA, pair.partB};
            // Общая точка приложения — середина между точками на двух поверхностях.
            row.point = Real(0.5) * (p.onA + p.onB);
            row.normal = pair.manifold.normal;
            row.separation = p.separation;
            row.restitution = std::max(a.restitution, b.restitution);
            row.normalMass = effectiveMass(a, b, row.point, row.normal);
            row.touching = pair.inContact() && p.separation <= kContactDistance;
            row.startSpeed = row.normal.dot(b.pointVelocity(row.point) - a.pointVelocity(row.point));
            row.roundA = pair.manifold.roundA, row.roundB = pair.manifold.roundB, row.faceOnB = pair.manifold.faceOnB;
            rows_.push_back(row);
        }
        pairs_.push_back(frictionOf(pair, first, rows_.size()));
    }
    if (joints_)
        for (const Joint& joint : *joints_)
            for (const Bond& bond : joint.bonds(bodies_, bodies_)) bonds_.push_back(bond);
    bondImpulse_.assign(bonds_.size(), 0), bondGoal_.assign(bonds_.size(), 0);
    for (const Bond& bond : bonds_) bondStart_.push_back(speed(bondLine(bond)));
    setDrift(bodies_);
}

// Ускорения точек касания без сил (geometricAcceleration) по скоростям тел moving — тех же тел в
// тех же позах. При создании решателя это сами тела; World уточняет их по средним за отрезок
// импульсам: угловая скорость тела, которое только что опёрлось, за шаг меняется сильно, и
// ускорение по скорости начала отрезка заметно ошибается.
Real ContactSolver::setDrift(const std::vector<Body>& moving) {
    Real largest = 0;
    for (Row& row : rows_) {
        const Body& a = moving[size_t(row.key.a)];
        const Body& b = moving[size_t(row.key.b)];
        // Нормаль поворачивается вместе с телом, которому принадлежит грань: вторая производная
        // зазора n · d равна n · d̈ + 2 ṅ · ḋ.
        const Vector3 turning = (row.faceOnB ? b : a).angularVelocity().cross(row.normal);
        const Vector3 relative = b.pointVelocity(row.point) - a.pointVelocity(row.point);
        const Vector3 coreA = row.point - row.roundA * row.normal, coreB = row.point + row.roundB * row.normal;
        row.drift = row.normal.dot(geometricAcceleration(b, coreB, row.point) - geometricAcceleration(a, coreA, row.point)) + 2 * turning.dot(relative);
        largest = std::max(largest, abs(row.drift));
    }
    for (Pair& pair : pairs_) {
        const Body& a = moving[size_t(pair.a)];
        const Body& b = moving[size_t(pair.b)];
        const Vector3 sliding = geometricAcceleration(b, pair.centre + pair.roundB * pair.normal, pair.centre) -
                             geometricAcceleration(a, pair.centre - pair.roundA * pair.normal, pair.centre);
        const Vector3 spinning = b.inverseInertiaTimes(b.angularMomentum.cross(b.angularVelocity())) - a.inverseInertiaTimes(a.angularMomentum.cross(a.angularVelocity()));
        pair.drift = {pair.tangent1.dot(sliding), pair.tangent2.dot(sliding), pair.reach > 0 ? pair.normal.dot(spinning) : Real(0)};
        for (Real d : pair.drift) largest = std::max(largest, abs(d));
    }
    if (joints_) {
        size_t k = 0;
        for (const Joint& joint : *joints_)
            for (const Bond& bond : joint.bonds(bodies_, moving)) largest = std::max(largest, abs(bonds_[k++].drift = bond.drift));
    }
    return largest;
}

// Трение пары: центр точек касания, матрица отклика и скольжение до действия сил.
ContactSolver::Pair ContactSolver::frictionOf(const ContactPair& contact, size_t first, size_t last) const {
    Pair pair{};
    pair.first = first, pair.last = last, pair.a = contact.a, pair.b = contact.b;
    pair.normal = contact.manifold.normal;
    tangents(pair.normal, pair.tangent1, pair.tangent2);
    pair.friction = sqrt(bodies_[size_t(pair.a)].friction * bodies_[size_t(pair.b)].friction);
    pair.roundA = contact.manifold.roundA, pair.roundB = contact.manifold.roundB;
    // Центр — центр давления касающихся точек; если давление неизвестно — их среднее; если
    // касающихся точек нет — среднее всех точек пары.
    bool anyTouching = false;
    for (size_t i = first; i < last; ++i) anyTouching = anyTouching || rows_[i].touching;
    std::vector<Real> weight(last - first, 0);
    Real total = 0;
    for (size_t i = first; i < last; ++i) {
        if (!rows_[i].touching || !pressure_) continue;
        const auto known = pressure_->find(rows_[i].key);
        if (known != pressure_->end()) total += weight[i - first] = std::max(Real(0), known->second.normal);
    }
    if (total == 0)
        for (size_t i = first; i < last; ++i) total += weight[i - first] = rows_[i].touching == anyTouching;
    for (size_t i = first; i < last; ++i) pair.centre += (weight[i - first] / total) * rows_[i].point;
    for (size_t i = first; i < last; ++i) pair.reach += (weight[i - first] / total) * (rows_[i].point - pair.centre).length();

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
ContactSolver::Response ContactSolver::frictionResponse(const Pair& pair, bool withA, bool withB) const {
    Response k{};
    const Vector3 tangent[2] = {pair.tangent1, pair.tangent2};
    const struct {
        int index;
        Real sign;
        bool moving;
    } sides[2] = {{pair.a, -1, withA}, {pair.b, 1, withB}};
    for (const auto& side : sides) {
        const Body& body = bodies_[size_t(side.index)];
        if (!side.moving || body.isStatic()) continue;
        const Vector3 r = pair.centre - body.position;
        for (size_t j = 0; j < 3; ++j) {
            // Отклик тела на единичный импульс j: изменение скорости и угловой скорости.
            const Vector3 linear = j < 2 ? (side.sign * body.mass.inverseMass) * tangent[j] : Vector3{};
            const Vector3 angular = side.sign * body.inverseInertiaTimes(j < 2 ? r.cross(tangent[j]) : pair.normal);
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
ContactSolver::Slip ContactSolver::slip(const Pair& pair) const {
    const Body& a = bodies_[size_t(pair.a)];
    const Body& b = bodies_[size_t(pair.b)];
    const Vector3 relative = b.pointVelocity(pair.centre) - a.pointVelocity(pair.centre);
    return {pair.tangent1.dot(relative), pair.tangent2.dot(relative),
            pair.reach > 0 ? pair.normal.dot(b.angularVelocity() - a.angularVelocity()) : Real(0)};
}

// Импульс трения пары: сила в центре и момент вокруг нормали, с противоположным знаком на два тела.
void ContactSolver::applyFriction(const Pair& pair, const Slip& impulse, bool toA, bool toB) {
    const Vector3 force = impulse[0] * pair.tangent1 + impulse[1] * pair.tangent2;
    Body& a = bodies_[size_t(pair.a)];
    Body& b = bodies_[size_t(pair.b)];
    if (toB && !b.isStatic()) b.applyImpulse(force, pair.centre), b.angularMomentum += impulse[2] * pair.normal;
    if (toA && !a.isStatic()) a.applyImpulse(-force, pair.centre), a.angularMomentum += (-impulse[2]) * pair.normal;
}

// Импульс трения, который гасит скольжение пары, в пределах закона Кулона. response и inverse —
// матрица отклика и обратная к ней; возвращает новый накопленный импульс.
ContactSolver::Slip ContactSolver::frictionImpulse(const Pair& pair, const std::vector<Real>& goal, const Response& inverse) const {
    // Предел силы — μ Σ λn; предел момента — μ Σ λn rᵢ, где rᵢ — расстояние точки от центра.
    Real load = 0, leverLoad = 0;
    for (size_t i = pair.first; i < pair.last; ++i) {
        if (goal[i] == kIdle) continue;
        load += sum_[i];
        leverLoad += sum_[i] * (rows_[i].point - pair.centre).length();
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
    if (holding_ && speed >= kSliding * kSliding && power > 0) {
        x[0] -= power * start[0] / speed;
        x[1] -= power * start[1] / speed;
        x[2] -= power * pair.reach * pair.reach * start[2] / speed;
    }
    const Real limit = pair.friction * load, size = sqrt(x[0] * x[0] + x[1] * x[1]);
    if (size > limit) x[0] *= limit / size, x[1] *= limit / size;
    x[2] = std::max(-pair.friction * leverLoad, std::min(pair.friction * leverLoad, x[2]));
    return x;
}

// Скорость тела b относительно тела a в точке контакта вдоль направления; отрицательная вдоль
// нормали — тела сближаются.
Real ContactSolver::relativeSpeed(const Row& row, Vector3 direction) const {
    return direction.dot(bodies_[size_t(row.key.b)].pointVelocity(row.point) - bodies_[size_t(row.key.a)].pointVelocity(row.point));
}

void ContactSolver::bondImpulses(std::vector<Vector3>& linear, std::vector<Vector3>& angular) const {
    linear.assign(bodies_.size(), {}), angular.assign(bodies_.size(), {});
    for (size_t j = 0; j < bonds_.size(); ++j) {
        const Bond& bond = bonds_[j];
        if (bond.driven || bond.soft) continue;
        if (!bodies_[size_t(bond.b)].isStatic()) linear[size_t(bond.b)] += bondImpulse_[j] * bond.linear, angular[size_t(bond.b)] += bondImpulse_[j] * bond.angularB;
        if (!bodies_[size_t(bond.a)].isStatic()) linear[size_t(bond.a)] += -bondImpulse_[j] * bond.linear, angular[size_t(bond.a)] += -bondImpulse_[j] * bond.angularA;
    }
}

// Импульс вдоль линии: телу b — сила linear и момент angularB, телу a — противоположные.
void ContactSolver::applyLine(const Line& line, Real impulse) {
    Body& a = bodies_[size_t(line.a)];
    Body& b = bodies_[size_t(line.b)];
    if (!b.isStatic()) b.linearMomentum += impulse * line.linear, b.angularMomentum += impulse * line.angularB;
    if (!a.isStatic()) a.linearMomentum += -impulse * line.linear, a.angularMomentum += -impulse * line.angularA;
}

void ContactSolver::apply(const Row& row, Vector3 impulse) {
    bodies_[size_t(row.key.b)].applyImpulse(impulse, row.point);
    bodies_[size_t(row.key.a)].applyImpulse(-impulse, row.point);
}

Real ContactSolver::kineticEnergy() const {
    Real total = 0;
    for (const Body& body : bodies_)
        if (!body.isStatic()) total += body.kineticEnergy();
    return total;
}

// Трение одной пары тел. Возвращает наибольшую поправку скорости.
Real ContactSolver::solveFriction(Pair& pair, const std::vector<Real>& goal) {
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

// Линия нормали в точке контакта.
ContactSolver::Line ContactSolver::normalLine(const Row& row) const {
    return {row.key.a, row.key.b, row.normal, (row.point - bodies_[size_t(row.key.a)].position).cross(row.normal),
            (row.point - bodies_[size_t(row.key.b)].position).cross(row.normal)};
}

// Линия трения пары: k = 0, 1 — вдоль касательных в центре, k = 2 — вращение вокруг нормали.
ContactSolver::Line ContactSolver::frictionLine(const Pair& pair, int k) const {
    if (k == 2) return {pair.a, pair.b, {}, pair.normal, pair.normal};
    const Vector3 d = k == 0 ? pair.tangent1 : pair.tangent2;
    return {pair.a, pair.b, d, (pair.centre - bodies_[size_t(pair.a)].position).cross(d), (pair.centre - bodies_[size_t(pair.b)].position).cross(d)};
}

Real ContactSolver::speed(const Line& line) const {
    const Body& a = bodies_[size_t(line.a)];
    const Body& b = bodies_[size_t(line.b)];
    return line.linear.dot(b.linearVelocity() - a.linearVelocity()) + line.angularB.dot(b.angularVelocity()) - line.angularA.dot(a.angularVelocity());
}

// Порядок неизвестных для разложения: остров за островом, внутри острова пары снизу вверх, у
// пары — её нормали, за ними её трение. Остров — пары, связанные общими подвижными телами;
// неподвижное тело островов не связывает. Между островами в матрице откликов стоят нули, а внутри
// стопки пара связана только с соседними, поэтому в таком порядке матрица блочно-диагональная с
// узкой лентой в каждом блоке — и разложение, которое считает только внутри профиля
// (math/ProfileMatrix.h), тратит время, пропорциональное числу неизвестных, а не его кубу.
std::vector<size_t> ContactSolver::eliminationOrder(const System& s) const {
    // Острова: тела объединяются по парам; метка острова — наименьший номер его тела.
    std::vector<int> island(bodies_.size());
    for (size_t i = 0; i < island.size(); ++i) island[i] = int(i);
    const auto root = [&](int body) {
        while (island[size_t(body)] != body) body = island[size_t(body)] = island[size_t(island[size_t(body)])];
        return body;
    };
    const auto join = [&](int x, int y) {
        if (bodies_[size_t(x)].isStatic() || bodies_[size_t(y)].isStatic()) return;
        const int a = root(x), b = root(y);
        island[size_t(std::max(a, b))] = std::min(a, b);
    };
    for (const Pair& pair : pairs_) join(pair.a, pair.b);
    for (const Bond& bond : bonds_) join(bond.a, bond.b);
    const auto islandOf = [&](int a, int b) { return root(bodies_[size_t(a)].isStatic() ? b : a); };

    // Блоки неизвестных: пары (уже упорядочены снизу вверх), за ними связи в порядке создания.
    // Устойчивая сортировка по острову сохраняет этот порядок внутри острова.
    struct Block {
        int island;
        bool bond;
        size_t index;
    };
    std::vector<Block> blocks;
    for (size_t p = 0; p < pairs_.size(); ++p) blocks.push_back({islandOf(pairs_[p].a, pairs_[p].b), false, p});
    for (size_t j = 0; j < bonds_.size(); ++j) blocks.push_back({islandOf(bonds_[j].a, bonds_[j].b), true, j});
    std::stable_sort(blocks.begin(), blocks.end(), [](const Block& x, const Block& y) { return x.island < y.island; });

    const size_t n = s.rows.size(), firstBond = n + 3 * s.pairs.size();
    std::vector<size_t> order;
    for (const Block& block : blocks) {
        if (block.bond) {
            order.push_back(firstBond + block.index);
            continue;
        }
        const size_t p = block.index;
        for (size_t row = pairs_[p].first; row < pairs_[p].last; ++row)
            if (s.rowPlace[row] < n) order.push_back(s.rowPlace[row]);
        if (s.pairPlace[p] < s.pairs.size())
            for (size_t k = 0; k < 3; ++k) order.push_back(n + 3 * s.pairPlace[p] + k);
    }
    return order;
}

// Накопленные импульсы и цели скорости по всем линиям системы: нормали, трение пар, связи. Пара,
// которая не скользит, за отрезок не должна сдвинуться. Без сил её точка касания сама ушла бы с
// ускорением drift, поэтому трение обязано оставить ей встречную скорость.
void ContactSolver::startTogether(const System& s, const std::vector<Real>& goal, std::vector<Real>& current, std::vector<Real>& target) const {
    const size_t n = s.rows.size(), firstBond = n + 3 * s.pairs.size();
    for (size_t i = 0; i < n; ++i) current[i] = sum_[s.rows[i]], target[i] = goal[s.rows[i]];
    for (size_t p = 0; p < s.pairs.size(); ++p) {
        for (size_t k = 0; k < 3; ++k) {
            current[n + 3 * p + k] = pairs_[s.pairs[p]].impulse[k];
            target[n + 3 * p + k] = -holdTime_ * pairs_[s.pairs[p]].drift[k];
        }
    }
    for (size_t j = 0; j < bonds_.size(); ++j) current[firstBond + j] = bondImpulse_[j], target[firstBond + j] = bondGoal_[j];
}

// Неизвестные: все нормали, линии трения несорвавшихся пар и связи без упора. Вращение вокруг
// нормали — только если у пары больше одной точки касания.
std::vector<char> ContactSolver::unknownsOf(const System& s) const {
    const size_t n = s.rows.size(), firstBond = n + 3 * s.pairs.size();
    std::vector<char> unknown(s.lines.size(), 0);
    for (size_t i = 0; i < n; ++i) unknown[i] = 1;
    for (size_t p = 0; p < s.pairs.size(); ++p)
        for (size_t k = 0; k < 3; ++k) unknown[n + 3 * p + k] = stuck_[s.pairs[p]] && (k < 2 || pairs_[s.pairs[p]].reach > 0);
    for (size_t j = 0; j < bonds_.size(); ++j) unknown[firstBond + j] = bondActive_[j] && !saturated_[j];
    return unknown;
}

// Запас скорости над целью вдоль линии i при импульсах λ равен free[i] + Σ A[i][j] λ[j]; free —
// запас, какой был бы без неизвестных импульсов вовсе. Слагаемое, удерживающее импульсы вблизи
// прежних: к диагонали добавляется ε Aᵢᵢ, к свободному члену — −ε Aᵢᵢ λᵢ.
std::vector<Real> ContactSolver::freeTerms(const System& s, const std::vector<char>& unknown, const std::vector<Real>& current, const std::vector<Real>& target) const {
    std::vector<Real> free(s.lines.size(), 0);
    for (size_t i = 0; i < s.lines.size(); ++i) {
        if (!unknown[i]) continue;
        free[i] = speed(s.lines[i]) - target[i] - kProximal * s.diagonal[i] * current[i];
        for (const Entry& entry : s.matrix[i])
            if (unknown[entry.column]) free[i] -= entry.value * current[entry.column];
    }
    return free;
}

// Предел Кулона для несорвавшихся пар: сила — μ Σ λn, момент — μ Σ λn rᵢ. Пара, вышедшая за
// предел, срывается; true — сорвалась хоть одна.
bool ContactSolver::releaseSlipping(const System& s, const std::vector<Real>& force) {
    const size_t n = s.rows.size();
    bool released = false;
    for (size_t p = 0; p < s.pairs.size(); ++p) {
        const Pair& pair = pairs_[s.pairs[p]];
        if (!stuck_[s.pairs[p]]) continue;
        Real load = 0, leverLoad = 0;
        for (size_t row = pair.first; row < pair.last; ++row) {
            if (s.rowPlace[row] == n) continue;
            load += std::max(Real(0), force[s.rowPlace[row]]);
            leverLoad += std::max(Real(0), force[s.rowPlace[row]]) * (rows_[row].point - pair.centre).length();
        }
        const Real* x = &force[n + 3 * p];
        const Real slack = 1 + Real(1e-12);
        if (sqrt(x[0] * x[0] + x[1] * x[1]) > pair.friction * load * slack || abs(x[2]) > pair.friction * leverLoad * slack)
            stuck_[s.pairs[p]] = 0, released = true;
    }
    return released;
}

// Предел силы привода. Привод, которому для заданной скорости нужен импульс больше предельного,
// прикладывает предельный и из неизвестных выходит. Упёршийся привод возвращается, если скорость
// перешла заданную: значит, полная сила уже не нужна. true — набор неизвестных изменился.
bool ContactSolver::releaseDrives(const System& s, const std::vector<char>& unknown, std::vector<Real>& current, const std::vector<Real>& target, const std::vector<Real>& force) {
    const size_t firstBond = s.rows.size() + 3 * s.pairs.size();
    bool released = false;
    for (size_t j = 0; j < bonds_.size(); ++j) {
        const Real cap = driveCap(j);
        const size_t i = firstBond + j;
        if (!bondActive_[j] || cap == std::numeric_limits<Real>::infinity()) continue;
        if (!saturated_[j]) {
            if (abs(force[i]) <= cap * (1 + Real(1e-12))) continue;
            saturated_[j] = force[i] > 0 ? 1 : -1;
            applyLine(s.lines[i], saturated_[j] * cap - bondImpulse_[j]);
            current[i] = bondImpulse_[j] = saturated_[j] * cap;
            released = true;
            continue;
        }
        Real excess = speed(s.lines[i]) - target[i];
        for (const Entry& entry : s.matrix[i])
            if (unknown[entry.column]) excess += entry.value * (force[entry.column] - current[entry.column]);
        if (saturated_[j] * excess > kSolved && bondFlips_[j]++ < kMaxDriveFlips) saturated_[j] = 0, released = true;
    }
    return released;
}

// Найденные импульсы прикладываются к телам; возвращает наибольшую поправку скорости.
Real ContactSolver::applyTogether(const System& s, const std::vector<Real>& force) {
    const size_t n = s.rows.size(), firstBond = n + 3 * s.pairs.size();
    Real worst = 0;
    for (size_t i = 0; i < n; ++i) {
        const Row& row = rows_[s.rows[i]];
        const Real normal = std::max(Real(0), force[i]);
        apply(row, (normal - sum_[s.rows[i]]) * row.normal);
        worst = std::max(worst, abs(normal - sum_[s.rows[i]]) / row.normalMass);
        sum_[s.rows[i]] = normal;
    }
    for (size_t p = 0; p < s.pairs.size(); ++p) {
        Pair& pair = pairs_[s.pairs[p]];
        if (!stuck_[s.pairs[p]]) continue;
        const Slip x = {force[n + 3 * p], force[n + 3 * p + 1], pair.reach > 0 ? force[n + 3 * p + 2] : Real(0)};
        const Slip change = {x[0] - pair.impulse[0], x[1] - pair.impulse[1], x[2] - pair.impulse[2]};
        applyFriction(pair, change, true, true);
        pair.impulse = x;
        worst = std::max({worst, abs(change[0]) * pair.response[0], abs(change[1]) * pair.response[1],
                          abs(change[2]) * pair.response[2] * pair.reach});
    }
    for (size_t j = 0; j < bonds_.size(); ++j) {
        if (!bondActive_[j] || saturated_[j]) continue;
        const Real impulse = bonds_[j].oneSided ? std::max(Real(0), force[firstBond + j]) : force[firstBond + j];
        const Real change = impulse - bondImpulse_[j];
        applyLine(s.lines[firstBond + j], change);
        bondImpulse_[j] = impulse;
        worst = std::max(worst, abs(change) * s.diagonal[firstBond + j]);
    }
    return worst;
}

// Точное решение сил опоры вместе с трением покоя. Неизвестные — нормальные импульсы и импульсы
// трения тех пар, которые не скользят: у них скольжение обязано остаться нулевым, это линейные
// равенства, и они решаются одной системой с нормалями. Пара, у которой нужное трение вышло за
// предел Кулона, срывается: её трение дальше считается отдельно (solveFriction). Перебор повторяется,
// пока срываются пары или упираются приводы. Возвращает наибольшую поправку скорости.
Real ContactSolver::solveTogether(const System& s, const std::vector<Real>& goal) {
    const size_t total = s.lines.size();
    std::vector<Real> current(total), target(total, 0), force(total);
    startTogether(s, goal, current, target);
    for (bool released = true; released;) {
        const std::vector<char> unknown = unknownsOf(s);
        ActiveSet(bodies_, s, bonds_, pressing_, bondPressing_).solve(unknown, freeTerms(s, unknown, current, target), force);
        released = releaseSlipping(s, force);
        if (releaseDrives(s, unknown, current, target, force)) released = true;
    }
    return applyTogether(s, force);
}


// Система задачи: нормали участвующих строк, трение пар, у которых такие строки есть, связи
// сочленений; места строк и пар в ней, матрица отклика, её диагональ и порядок исключения.
ContactSolver::System ContactSolver::assemble(const std::vector<Real>& goal) {
    System s;
    for (size_t i = 0; i < rows_.size(); ++i)
        if (goal[i] != kIdle) s.rows.push_back(i), s.lines.push_back(normalLine(rows_[i]));
    // Начальное предположение: давят точки, где уже есть сила (тёплый старт) или где без неё
    // тела сблизились бы больше допуска. Точка, которой до цели не хватает 10⁻¹² м/с, не давит:
    // у давящей точки скорость обязана сравняться с целью точно, и если тела заперты трением покоя
    // (кубы пирамиды касаются боками и стоят на полу), такому требованию не может ответить никакая
    // сила — решение уходило по этой точке всё дальше с каждым кругом.
    pressing_.assign(rows_.size(), 0);
    for (size_t i : s.rows) pressing_[i] = sum_[i] > 0 || relativeSpeed(rows_[i], rows_[i].normal) < goal[i] - kSolved;
    addFriction(s, goal);
    addBonds(s);
    s.rowPlace.assign(rows_.size(), s.rows.size());
    for (size_t i = 0; i < s.rows.size(); ++i) s.rowPlace[s.rows[i]] = i;
    s.pairPlace.assign(pairs_.size(), s.pairs.size());
    for (size_t q = 0; q < s.pairs.size(); ++q) s.pairPlace[s.pairs[q]] = q;
    s.matrix = ContactSystem::responseMatrix(bodies_, s.lines);
    for (size_t i = 0; i < s.lines.size(); ++i)
        for (const Entry& entry : s.matrix[i])
            if (entry.column == i) s.diagonal.push_back(entry.value);
    s.order = eliminationOrder(s);
    return s;
}

// Три линии трения на пару с участвующими строками. Не скользит пара, которая покоилась в начале
// отрезка; при ударе — любая, пока трения хватает.
void ContactSolver::addFriction(System& s, const std::vector<Real>& goal) {
    stuck_.assign(pairs_.size(), 0);
    for (size_t p = 0; p < pairs_.size(); ++p) {
        const Pair& pair = pairs_[p];
        bool active = false;
        for (size_t i = pair.first; i < pair.last; ++i) active = active || goal[i] != kIdle;
        if (!active || pair.friction == 0) continue;
        s.pairs.push_back(p);
        for (int k = 0; k < 3; ++k) s.lines.push_back(frictionLine(pair, k));
        const Slip& start = pair.startSlip;
        const Real speed = start[0] * start[0] + start[1] * start[1] + pair.reach * pair.reach * start[2] * start[2];
        stuck_[p] = !holding_ || speed < kSliding * kSliding;
    }
}

// Линии связей сочленений. Предел хода, до которого сочленение не дошло, не действует; привод с
// конечной силой за нулевое время импульса не даёт. Сочленение давит на предел, если без его
// силы прошло бы дальше. Пружина с демпфером — линия с податливостью: импульс
// λ = −h (k (e + h v) + c v) даёт конечную скорость v = −γ λ − γ h k e, γ = 1 / (h² k + h c);
// к диагонали добавляется γ, цель — −γ h k e.
void ContactSolver::addBonds(System& s) {
    for (const Bond& bond : bonds_) s.lines.push_back(bondLine(bond));
    bondActive_.assign(bonds_.size(), 1), bondPressing_.assign(bonds_.size(), 0);
    saturated_.assign(bonds_.size(), 0), bondFlips_.assign(bonds_.size(), 0);
    s.soft.assign(s.lines.size(), 0);
    for (size_t j = 0; j < bonds_.size(); ++j)
        if (bonds_[j].soft && holdTime_ > 0) s.soft[s.lines.size() - bonds_.size() + j] = 1 / (holdTime_ * holdTime_ * bonds_[j].stiffness + holdTime_ * bonds_[j].damping);
    for (size_t j = 0; j < bonds_.size(); ++j) {
        // За нулевое время пружина и демпфер импульса не дают.
        bondActive_[j] = !bonds_[j].idle && driveCap(j) > 0 && !(bonds_[j].soft && holdTime_ == 0);
        bondPressing_[j] = bonds_[j].oneSided && speed(bondLine(bonds_[j])) < bondGoal_[j];
    }
}

void ContactSolver::solve(const std::vector<Real>& goal) {
    const System s = assemble(goal);
    // Скользящие и сорвавшиеся пары обходятся по очереди после точного решения остальных. Круги
    // повторяются, пока условия задачи не выполнены с допуском (residual) — или пока поправки не
    // перестанут убывать: при больших отношениях масс округление не даёт им опуститься ниже
    // своего уровня, и дальше круги ничего не меняют.
    //
    // Мерка сходимости — невязка скорости, а не изменение импульсов. В вырожденной задаче импульсы
    // могут меняться от круга к кругу, не меняя скоростей: у пирамиды из 36 кубов круги шли до
    // правила «не убывают» при скоростях, выполненных с первого круга, а импульс ненагруженной
    // точки рос на одну и ту же величину каждый круг.
    Real best = std::numeric_limits<Real>::infinity();
    int stalled = 0;
    for (iterations_ = 1; iterations_ <= kMaxIterations; ++iterations_) {
        Real worst = solveTogether(s, goal), rubbing = 0;
        for (size_t p = 0; p < s.pairs.size(); ++p)
            if (!stuck_[s.pairs[p]]) rubbing = std::max(rubbing, solveFriction(pairs_[s.pairs[p]], goal));
        for (size_t p = s.pairs.size(); p-- > 0;)
            if (!stuck_[s.pairs[p]]) rubbing = std::max(rubbing, solveFriction(pairs_[s.pairs[p]], goal));
        if (rubbing < kSolved && residual(s, goal) < kSolved) break;
        worst = std::max(worst, rubbing);
        if (worst < kSolved) break;
        if (worst < best / 2) best = worst, stalled = 0;
        else if (++stalled == 5) break;
    }
}

// Невязка точной части задачи при нынешних импульсах, в единицах скорости: насколько скорость
// давящей точки отличается от цели, а недавящей — меньше её; насколько скользит пара, которую
// держит трение покоя; насколько не выполнены связи сочленений.
Real ContactSolver::residual(const System& s, const std::vector<Real>& goal) const {
    Real worst = 0;
    for (size_t i : s.rows) {
        const Real excess = relativeSpeed(rows_[i], rows_[i].normal) - goal[i];
        worst = std::max(worst, sum_[i] > 0 ? abs(excess) : -excess);
    }
    for (size_t q : s.pairs) {
        if (!stuck_[q]) continue;
        const Slip now = slip(pairs_[q]);
        for (size_t k = 0; k < 3; ++k) worst = std::max(worst, abs(now[k] + holdTime_ * pairs_[q].drift[k]) * (k == 2 ? pairs_[q].reach : Real(1)));
    }
    const size_t firstBond = s.lines.size() - bonds_.size();
    for (size_t j = 0; j < bonds_.size(); ++j) {
        if (!bondActive_[j] || saturated_[j]) continue;
        // У податливой линии цель — для скорости вместе с её собственной подачей.
        const Real excess = speed(s.lines[firstBond + j]) + s.soft[firstBond + j] * bondImpulse_[j] - bondGoal_[j];
        worst = std::max(worst, bonds_[j].oneSided && bondImpulse_[j] <= 0 ? -excess : abs(excess));
    }
    return worst;
}

Real ContactSolver::driveCap(size_t bond) const {
    const Real strength = bonds_[bond].strength;
    return strength == std::numeric_limits<Real>::infinity() ? strength : strength * holdTime_;
}

void ContactSolver::solveImpacts() {
    // Отскок определяется скоростью, с которой тела пришли, поэтому цели считаются до решения.
    std::vector<Real> goal(rows_.size(), kIdle);
    for (size_t i = 0; i < rows_.size(); ++i) {
        if (!rows_[i].touching) continue;
        // Отскок, который не вывел бы тело из полосы касания, — не отскок: иначе тело,
        // приходящее в покой, отскакивало бы бесконечно много раз за конечное время.
        const Real approach = rows_[i].startSpeed;
        goal[i] = approach < -restBounceSpeed_ ? -rows_[i].restitution * approach : Real(0);
    }
    reset();
    holding_ = false;
    holdTime_ = 0;
    // Скорость вдоль запертой свободы — нулевая, вдоль свободы с приводом — заданная.
    for (size_t j = 0; j < bonds_.size(); ++j) bondGoal_[j] = bonds_[j].speed;
    solve(goal);
}

ContactForces ContactSolver::pressures() const {
    ContactForces known;
    for (size_t i = 0; i < rows_.size(); ++i)
        if (rows_[i].touching) known[rows_[i].key] = {sum_[i], {}, 0};
    return known;
}

void ContactSolver::reset() {
    sum_.assign(rows_.size(), 0);
    for (Pair& pair : pairs_) pair.impulse = {};
    std::fill(bondImpulse_.begin(), bondImpulse_.end(), Real(0));
}

void ContactSolver::solveOverlap() {
    std::vector<Real> goal(rows_.size(), kIdle);
    for (size_t i = 0; i < rows_.size(); ++i)
        if (rows_[i].touching) goal[i] = rows_[i].separation < -kOverlap / 2 ? -rows_[i].separation : Real(0);
    reset();
    holding_ = false;
    // Разошедшаяся связь за единицу времени возвращается на место; пройденный предел хода — тоже,
    // а не пройденный не даёт к себе подойти ближе, чем осталось.
    for (size_t j = 0; j < bonds_.size(); ++j) bondGoal_[j] = bonds_[j].oneSided ? -bonds_[j].gap : -bonds_[j].error;
    solve(goal);
}

// Прошлые силы, приложенные к нынешним скоростям. Изменение кинетической энергии от импульсов
// α λ равно α c + α² q / 2, где c = λ · (J v), q = λ · A λ. Наименьшее оно при α = −c / q; больше
// единицы масштаб не берётся — прошлые силы не преувеличиваются.
void ContactSolver::warmStart(const std::vector<Real>& goal, const ContactForces& previous, Real h) {
    Real c = 0;
    for (Pair& pair : pairs_) {
        for (size_t i = pair.first; i < pair.last; ++i) {
            const auto found = previous.find(rows_[i].key);
            if (goal[i] == kIdle || !rows_[i].touching || found == previous.end()) continue;
            sum_[i] = h * found->second.normal;
            c += sum_[i] * relativeSpeed(rows_[i], rows_[i].normal);
            pair.impulse[0] += h * found->second.tangent.dot(pair.tangent1);
            pair.impulse[1] += h * found->second.tangent.dot(pair.tangent2);
            pair.impulse[2] += h * found->second.twist;
        }
        const Slip now = slip(pair);
        for (size_t k = 0; k < 3; ++k) c += pair.impulse[k] * now[k];
    }
    const auto applyScaled = [&](Real scale) {
        for (size_t i = 0; i < rows_.size(); ++i) apply(rows_[i], (scale * sum_[i]) * rows_[i].normal);
        for (const Pair& pair : pairs_) applyFriction(pair, {scale * pair.impulse[0], scale * pair.impulse[1], scale * pair.impulse[2]}, true, true);
    };
    const Real before = kineticEnergy();
    applyScaled(1);
    const Real q = 2 * (kineticEnergy() - before - c);
    const Real alpha = q > 0 ? std::min(Real(1), std::max(Real(0), -c / q)) : Real(c <= 0);
    if (alpha < 1) {
        applyScaled(alpha - 1);
        for (Real& normal : sum_) normal *= alpha;
        for (Pair& pair : pairs_) pair.impulse = {alpha * pair.impulse[0], alpha * pair.impulse[1], alpha * pair.impulse[2]};
    }
}

void ContactSolver::propagateShock(const std::vector<Real>& goal) {
    const Body frozen; // неподвижное тело: обратные масса и инерция нулевые
    for (Pair& pair : pairs_) { // пары уже упорядочены снизу вверх
        Body& a = bodies_[size_t(pair.a)];
        Body& b = bodies_[size_t(pair.b)];
        const bool upperIsB = up_.dot(b.position - a.position) > 0;
        Body& upper = upperIsB ? b : a;
        if (upper.isStatic()) continue;
        const Real sign = upperIsB ? Real(1) : Real(-1); // импульс +J получает тело b, −J — тело a
        bool active = false;
        for (size_t i = pair.first; i < pair.last; ++i) {
            if (goal[i] == kIdle) continue;
            active = true;
            const Row& row = rows_[i];
            const Real approaching = goal[i] - relativeSpeed(row, row.normal);
            if (approaching > 0) upper.applyImpulse((sign * approaching * effectiveMass(frozen, upper, row.point, row.normal)) * row.normal, row.point);
        }
        if (!active) continue;
        const Slip x = frictionImpulse(pair, goal, inverted(frictionResponse(pair, !upperIsB, upperIsB)));
        // Поправка прикладывается только к верхнему телу и в запоминаемую силу трения не входит:
        // иначе тёплый старт следующего отрезка приложил бы её к обоим телам.
        applyFriction(pair, {x[0] - pair.impulse[0], x[1] - pair.impulse[1], x[2] - pair.impulse[2]}, !upperIsB, upperIsB);
    }
}

ContactForces ContactSolver::solveRestingContacts(Real h, const ContactForces& previous, bool shockPropagation) {
    std::vector<Real> goal(rows_.size(), kIdle);
    for (size_t i = 0; i < rows_.size(); ++i) {
        if (rows_[i].touching) goal[i] = -rows_[i].startSpeed - h * rows_[i].drift;
    }
    reset();
    holding_ = true;
    holdTime_ = h;
    // Связь за отрезок не должна разойтись: как и у точки контакта, перемещение равно средней из
    // скоростей в начале и в конце, умноженной на h, плюс уход от вращения тел.
    for (size_t j = 0; j < bonds_.size(); ++j) {
        const Bond& bond = bonds_[j];
        bondGoal_[j] = bond.soft ? -h * bond.stiffness * bond.stretch / (h * h * bond.stiffness + h * bond.damping)
                                 : bond.driven ? bond.speed : -bondStart_[j] - h * bond.drift + (j < bondBias_.size() ? bondBias_[j] : Real(0));
    }
    warmStart(goal, previous, h);
    solve(goal);
    if (shockPropagation) propagateShock(goal);

    // Сила трения пары запоминается поровну на её касающихся точках: при тёплом старте доли
    // складываются обратно.
    ContactForces forces;
    for (const Pair& pair : pairs_) {
        int touching = 0;
        for (size_t i = pair.first; i < pair.last; ++i) touching += rows_[i].touching;
        const Real share = touching ? 1 / (h * touching) : Real(0);
        for (size_t i = pair.first; i < pair.last; ++i)
            if (rows_[i].touching)
                forces[rows_[i].key] = {sum_[i] / h, share * (pair.impulse[0] * pair.tangent1 + pair.impulse[1] * pair.tangent2), share * pair.impulse[2]};
    }
    return forces;
}

void ContactSolver::settle(SlidingStop stopped) {
    if (!stopped.halted) return;
    for (Pair& pair : pairs_)
        if (pair.a == stopped.a && pair.b == stopped.b) pair.startSlip = {};
}

ContactSolver::SlidingStop ContactSolver::slidingStop(Real h, const std::vector<std::pair<int, int>>& already) const {
    SlidingStop earliest{h};
    for (const Pair& pair : pairs_) {
        if (std::find(already.begin(), already.end(), std::make_pair(pair.a, pair.b)) != already.end()) continue;
        bool touching = false;
        for (size_t i = pair.first; i < pair.last; ++i) touching = touching || rows_[i].touching;
        if (!touching || pair.friction == 0) continue;
        // Скольжение s(τ) = s₀ + τ (s₁ − s₀), τ от 0 до 1; вращение вокруг нормали переведено в
        // скорость на расстоянии reach от центра. Оно развернулось, если s₀ · s₁ < 0; ближе всего
        // к нулю оно при τ = s₀ · (s₀ − s₁) / |s₀ − s₁|².
        const Slip now = slip(pair);
        const Real start[3] = {pair.startSlip[0], pair.startSlip[1], pair.reach * pair.startSlip[2]};
        const Real end[3] = {now[0], now[1], pair.reach * now[2]};
        Real speed = 0, along = 0, changed = 0, projected = 0;
        for (int k = 0; k < 3; ++k) {
            speed += start[k] * start[k];
            along += start[k] * end[k];
            changed += (start[k] - end[k]) * (start[k] - end[k]);
            projected += start[k] * (start[k] - end[k]);
        }
        if (speed < kSliding * kSliding || along >= 0) continue;
        // Квадрат наименьшего скольжения на отрезке: остановка — если оно мало рядом с начальным.
        const Real least = speed - projected * projected / changed;
        // Остановка в самом конце отрезка событием не является: отрезок и так кончается на ней.
        if (projected >= changed * (1 - Real(1e-9))) continue;
        if (h * projected / changed < earliest.time) earliest = {h * projected / changed, pair.a, pair.b, least <= kHalted * kHalted * speed};
    }
    return earliest;
}

} // namespace kp
