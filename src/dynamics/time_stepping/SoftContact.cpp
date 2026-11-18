#include <kp/dynamics/time_stepping/SoftContact.h>

#include <kp/math/Complementarity.h>

#include <algorithm>
#include <cmath>

namespace kp {
namespace {

constexpr Real kSpeculative = Real(0.02); // м: точки дальше этого на центр трения почти не влияют
constexpr Real kLeastWeight = Real(1e-3);
constexpr Real kEquallyDeep = Real(1e-3); // м: точки, чьи глубины ближе этого, считаются одинаково глубокими

// Четыре точки, которых хватает опоре: самая глубокая, самая далёкая от неё, та, что даёт с ними
// наибольший треугольник, и та, что добавляет наибольшую площадь по другую сторону. Так же отбирают
// точки Box3D, Jolt и ReactPhysics3D.
std::vector<int> fourOf(const ContactManifold& manifold) {
    const auto at = [&](int k) { return manifold.points[size_t(k)].onA; };
    int first = 0;
    for (int k = 1; k < manifold.count; ++k)
        if (manifold.points[size_t(k)].separation < manifold.points[size_t(first)].separation - kEquallyDeep) first = k;
    int second = first;
    for (int k = 0; k < manifold.count; ++k)
        if ((at(k) - at(first)).lengthSquare() > (at(second) - at(first)).lengthSquare()) second = k;
    const auto area = [&](int k) { return (at(second) - at(first)).cross(at(k) - at(first)).dot(manifold.normal); };
    int third = first, fourth = first;
    for (int k = 0; k < manifold.count; ++k) {
        if (area(k) > area(third)) third = k;
        if (area(k) < area(fourth)) fourth = k;
    }
    std::vector<int> kept;
    for (int k : {first, second, third, fourth})
        if (std::find(kept.begin(), kept.end(), k) == kept.end()) kept.push_back(k);
    std::sort(kept.begin(), kept.end());
    return kept;
}

// Рабочие массивы решения площадки живут в потоке: проход по контакту не выделяет память.
struct BlockWork {
    std::vector<Vector3> armA, armB, spinA, spinB; // r × n и I⁻¹ (r × n): поворот от единичного импульса в точке
    std::vector<Real> matrix, q, scale, unscaled;
};
thread_local BlockWork work;
constexpr std::uint32_t kEdgePair = 0xC0000000u; // номер точки пары рёбер (HullVsHullAlgorithm::clipFaces)

} // namespace

SoftContact::SoftContact(const ContactPair& pair, const std::vector<Body>& bodies, const std::vector<SolverBody>& states, const Softness& soft, const ContactForces& memory, Real h, bool fourPoints)
    : a_(pair.a), b_(pair.b), normal_(pair.manifold.normal), roundA_(pair.manifold.roundA), roundB_(pair.manifold.roundB), faceOnB_(pair.manifold.faceOnB), soft_(soft) {
    const Body &bodyA = bodies[size_t(pair.a)], &bodyB = bodies[size_t(pair.b)];
    const SolverBody &a = states[size_t(pair.a)], &b = states[size_t(pair.b)];
    const Vector3 n = normal_;
    const Vector3 aside = abs(n.x) < Real(0.57) ? Vector3{1, 0, 0} : Vector3{0, 1, 0};
    tangent1_ = n.cross(aside), tangent1_ = (1 / tangent1_.length()) * tangent1_, tangent2_ = n.cross(tangent1_);
    friction_ = sqrt(bodyA.friction * bodyB.friction), restitution_ = std::max(bodyA.restitution, bodyB.restitution);
    Real weights = 0;
    Vector3 rubbed{};
    std::vector<int> kept;
    if (fourPoints && pair.manifold.count > 4) kept = fourOf(pair.manifold);
    else
        for (int k = 0; k < pair.manifold.count; ++k) kept.push_back(k);
    for (int k : kept) {
        const ContactPoint& contact = pair.manifold.points[size_t(k)];
        Point p{};
        p.key = {pair.a, pair.b, contact.id, pair.partA, pair.partB};
        const Vector3 at = Real(0.5) * (contact.onA + contact.onB);
        p.rA = at - bodyA.position, p.rB = at - bodyB.position;
        p.coreA = p.rA - roundA_ * n, p.coreB = p.rB + roundB_ * n;
        p.base = contact.separation;
        const Real response = SolverBody::inverseMassAlong(a, b, p.rA, p.rB, n);
        p.normalMass = response > 0 ? 1 / response : 0;
        p.approach = n.dot(b.v + b.w.cross(p.rB) - a.v - a.w.cross(p.rA));
        const auto known = memory.find(p.key);
        if (known != memory.end()) p.impulse = h * known->second.normal, rubbed += h * known->second.tangent, twistImpulse_ += h * known->second.twist;
        const Real weight = std::max(kLeastWeight, std::min(Real(1), 2 - contact.separation / kSpeculative));
        centreA_ += weight * p.rA, centreB_ += weight * p.rB, weights += weight;
        points_.push_back(p);
    }
    centreA_ = (1 / weights) * centreA_, centreB_ = (1 / weights) * centreB_;
    centreCoreA_ = centreA_ - roundA_ * n, centreCoreB_ = centreB_ + roundB_ * n;
    normal0_ = normal_, tangent10_ = tangent1_, tangent20_ = tangent2_, apart_ = bodyB.position - bodyA.position;
    edges_ = points_.size() == 1 && roundA_ == 0 && roundB_ == 0 && (points_[0].key.id & kEdgePair) == kEdgePair;
    for (Point& p : points_) p.lever = (p.rA - centreA_).length();
    rub_ = {rubbed.dot(tangent1_), rubbed.dot(tangent2_)};
    // Отклик трения в центре: матрица 2 × 2 и её обратная.
    const Vector3 a1 = centreA_.cross(tangent1_), a2 = centreA_.cross(tangent2_), b1 = centreB_.cross(tangent1_), b2 = centreB_.cross(tangent2_);
    const Real k11 = a.inverseMass + b.inverseMass + a1.dot(a.spinFrom(a1)) + b1.dot(b.spinFrom(b1));
    const Real k22 = a.inverseMass + b.inverseMass + a2.dot(a.spinFrom(a2)) + b2.dot(b.spinFrom(b2));
    const Real k12 = a1.dot(a.spinFrom(a2)) + b1.dot(b.spinFrom(b2));
    tangentMass_ = Matrix2x2{k11, k12, k12, k22}.inverse();
    const Real around = n.dot(a.spinFrom(n) + b.spinFrom(n));
    twistMass_ = around > 0 ? 1 / around : 0;
    const Vector3 turn1 = a.spinFrom(tangent1_) + b.spinFrom(tangent1_), turn2 = a.spinFrom(tangent2_) + b.spinFrom(tangent2_);
    rollMass_ = Matrix2x2{tangent1_.dot(turn1), tangent1_.dot(turn2), tangent1_.dot(turn2), tangent2_.dot(turn2)}.inverse();
}

void SoftContact::follow(const std::vector<SolverBody>& states) {
    const SolverBody &a = states[size_t(a_)], &b = states[size_t(b_)];
    if (roundA_ > 0 && roundB_ > 0) {
        const Vector3 between = apart_ + b.moved - a.moved + b.turned.rotate(points_[0].coreB) - a.turned.rotate(points_[0].coreA);
        if (between.length() > 0) normal_ = (1 / between.length()) * between;
        tangent1_ = tangent10_ - tangent10_.dot(normal_) * normal_, tangent1_ = (1 / tangent1_.length()) * tangent1_, tangent2_ = normal_.cross(tangent1_);
    } else {
        const Real same = a.turned.w * b.turned.w + a.turned.x * b.turned.x + a.turned.y * b.turned.y + a.turned.z * b.turned.z < 0 ? Real(-1) : Real(1);
        const Quaternion mean = Quaternion{a.turned.w + same * b.turned.w, a.turned.x + same * b.turned.x, a.turned.y + same * b.turned.y, a.turned.z + same * b.turned.z}.unit();
        const Quaternion turn = edges_ ? mean : faceOnB_ ? b.turned : a.turned;
        normal_ = turn.rotate(normal0_), tangent1_ = turn.rotate(tangent10_), tangent2_ = turn.rotate(tangent20_);
    }
    centreA_ = a.turned.rotate(centreCoreA_) + roundA_ * normal_, centreB_ = b.turned.rotate(centreCoreB_) - roundB_ * normal_;
    for (Point& p : points_) p.rA = a.turned.rotate(p.coreA) + roundA_ * normal_, p.rB = b.turned.rotate(p.coreB) - roundB_ * normal_;
}

void SoftContact::warmStart(std::vector<SolverBody>& states) const {
    SolverBody &a = states[size_t(a_)], &b = states[size_t(b_)];
    for (const Point& p : points_) SolverBody::push(a, b, p.rA, p.rB, p.impulse * normal_);
    SolverBody::twist(a, b, twistImpulse_ * normal_);
    SolverBody::push(a, b, centreA_, centreB_, rub_.x * tangent1_ + rub_.y * tangent2_);
    if (roll_.x != 0 || roll_.y != 0) SolverBody::twist(a, b, roll_.x * tangent1_ + roll_.y * tangent2_);
}

// K — отклик нынешних тел: у вытянутого тела тензор инерции в мире меняется за кадр, и точное
// решение по K начала кадра раскачивало цепь брусков до 10⁵ м/с.
void SoftContact::respond(const SolverBody& a, const SolverBody& b) const {
    const size_t count = points_.size();
    BlockWork& w = work;
    w.armA.resize(count), w.armB.resize(count), w.spinA.resize(count), w.spinB.resize(count);
    w.matrix.resize(count * count), w.q.resize(count), w.scale.assign(count, 1), w.unscaled.resize(count);
    for (size_t i = 0; i < count; ++i) {
        w.armA[i] = points_[i].rA.cross(normal_), w.armB[i] = points_[i].rB.cross(normal_);
        w.spinA[i] = a.spinFrom(w.armA[i]), w.spinB[i] = b.spinFrom(w.armB[i]);
    }
    for (size_t i = 0; i < count; ++i)
        for (size_t j = 0; j <= i; ++j)
            w.matrix[i * count + j] = w.matrix[j * count + i] = a.inverseMass + b.inverseMass + w.armA[i].dot(w.spinA[j]) + w.armB[i].dot(w.spinB[j]);
}

void SoftContact::solveNormals(std::vector<SolverBody>& states, bool withBias, Real h, Real pushSpeed) {
    SolverBody &a = states[size_t(a_)], &b = states[size_t(b_)];
    const Vector3 n = normal_, shift = b.moved - a.moved;
    const size_t count = points_.size();
    BlockWork& w = work;
    respond(a, b);
    for (size_t i = 0; i < count; ++i) {
        const Point& p = points_[i];
        const Real gap = (shift + p.rB - p.rA + apart_).dot(n) + p.base;
        Real bias = 0;
        if (gap > 0) bias = gap / h; // до касания ещё далеко: сближаться можно не быстрее gap / h
        else if (withBias) w.scale[i] = soft_.massScale, bias = std::max(soft_.massScale * soft_.biasRate * gap, -pushSpeed) / soft_.massScale;
        w.q[i] = n.dot(b.v - a.v) + b.w.dot(w.armB[i]) - a.w.dot(w.armA[i]) + bias; // n · (ω × r) = ω · (r × n)
        for (size_t j = 0; j < count; ++j) w.q[i] -= w.matrix[i * count + j] * points_[j].impulse;
    }
    // Проксимальная поправка: у площадки с лишними опорами одно равновесие держат многие наборы сил,
    // и точное решение выбирало набор заново каждый проход — пирамида дрожала, цепь взрывалась.
    // Штраф ε Kᵢᵢ (μᵢ − μᵢ прошл)² делает ответ единственным и ближайшим к прошлому.
    // ε = 0,3 — по замеру ε ∈ {0,1; 0,3; 1; 3}: меньше — сдвиг пирамиды 1 мм, больше — куб 100:1 не
    // успокаивается (ε = 1) и возвращается дрожь (ε = 3: сотни разворотов скорости).
    // У одной точки равносильных наборов нет, и поправка только замедляла бы сходимость (шар в
    // ведре не успокаивался): с одной точкой это в точности ответ Box3D.
    constexpr Real kProximal = Real(0.3);
    const Real proximal = count > 1 ? kProximal : 0;
    for (size_t i = 0; i < count; ++i) {
        const Real pull = proximal * w.matrix[i * count + i];
        w.unscaled[i] = points_[i].impulse / w.scale[i]; // прошлый ответ — тёплый старт набора давящих точек
        w.matrix[i * count + i] += pull, w.q[i] -= pull * w.unscaled[i];
    }
    solveComplementarity(w.matrix, w.q, w.unscaled);
    for (size_t i = 0; i < count; ++i) {
        Point& p = points_[i];
        const Real next = w.scale[i] * w.unscaled[i], change = next - p.impulse;
        a.v -= (a.inverseMass * change) * n, a.w -= change * w.spinA[i];
        b.v += (b.inverseMass * change) * n, b.w += change * w.spinB[i];
        p.impulse = next;
        if (!withBias) p.largest = std::max(p.largest, next);
    }
}

void SoftContact::solveFriction(std::vector<SolverBody>& states) {
    SolverBody &a = states[size_t(a_)], &b = states[size_t(b_)];
    Real load = 0, leverLoad = 0;
    for (const Point& p : points_) load += p.impulse, leverLoad += p.lever * p.impulse;
    const Real turning = std::max(-friction_ * leverLoad, std::min(friction_ * leverLoad, twistImpulse_ - twistMass_ * normal_.dot(b.w - a.w)));
    SolverBody::twist(a, b, (turning - twistImpulse_) * normal_);
    twistImpulse_ = turning;
    const Vector3 slip = b.v + b.w.cross(centreB_) - a.v - a.w.cross(centreA_);
    const Real s1 = slip.dot(tangent1_), s2 = slip.dot(tangent2_);
    Vector2 rub = rub_ - tangentMass_ * Vector2{s1, s2};
    const Real size = rub.length(), limit = friction_ * load;
    if (size > limit) rub = (limit / size) * rub;
    SolverBody::push(a, b, centreA_, centreB_, (rub.x - rub_.x) * tangent1_ + (rub.y - rub_.y) * tangent2_);
    rub_ = rub;
}

void SoftContact::solveRolling(std::vector<SolverBody>& states, Real arm) {
    if (arm == 0) return;
    SolverBody &a = states[size_t(a_)], &b = states[size_t(b_)];
    Real load = 0;
    for (const Point& p : points_) load += p.impulse;
    const Vector3 spin = b.w - a.w;
    Vector2 roll = roll_ - rollMass_ * Vector2{spin.dot(tangent1_), spin.dot(tangent2_)};
    const Real size = roll.length(), limit = arm * load;
    if (size > limit) roll = (limit / size) * roll;
    SolverBody::twist(a, b, (roll.x - roll_.x) * tangent1_ + (roll.y - roll_.y) * tangent2_);
    roll_ = roll;
}

void SoftContact::bounce(std::vector<SolverBody>& states, Real bounceSpeed) {
    if (restitution_ == 0) return;
    SolverBody &a = states[size_t(a_)], &b = states[size_t(b_)];
    for (Point& p : points_) {
        if (p.approach > -bounceSpeed || p.largest == 0) continue;
        const Real speed = normal_.dot(b.v + b.w.cross(p.rB) - a.v - a.w.cross(p.rA));
        const Real next = std::max(p.impulse - p.normalMass * (speed + restitution_ * p.approach), Real(0));
        SolverBody::push(a, b, p.rA, p.rB, (next - p.impulse) * normal_);
        p.impulse = next;
    }
}

void SoftContact::remember(ContactForces& memory, Real h) const {
    const Real share = 1 / (h * Real(points_.size()));
    for (const Point& p : points_) memory[p.key] = {p.impulse / h, share * (rub_.x * tangent1_ + rub_.y * tangent2_), share * twistImpulse_};
}

} // namespace kp
