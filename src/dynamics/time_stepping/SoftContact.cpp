#include <math/StepMemory.h>
#include <dynamics/time_stepping/SoftContact.h>

#include <collision/narrowphase/Gjk.h>
#include <math/Complementarity.h>

#include <algorithm>
#include <numeric>
#include <cmath>

namespace kp {
namespace {

constexpr Real kSpeculative = Real(0.02); // м: точки дальше этого на центр трения почти не влияют
constexpr Real kLeastWeight = Real(1e-3);
constexpr Real kEquallyDeep = Real(1e-3); // м: точки, чьи глубины ближе этого, считаются одинаково глубокими

// Четыре точки, которых хватает опоре: самая глубокая, самая далёкая от неё, та, что даёт с ними
// наибольший треугольник, и та, что добавляет наибольшую площадь по другую сторону. Так же отбирают
// точки Box3D, Jolt и ReactPhysics3D.
StepArray<int> fourOf(const ContactManifold& manifold) {
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
    StepArray<int> kept;
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

// Средний поворот двух тел: кватернионы складываются в одном полушарии. Нужен только паре рёбер.
Quaternion meanTurn(const Quaternion& p, const Quaternion& q) {
    const Real same = p.w * q.w + p.x * q.x + p.y * q.y + p.z * q.z < 0 ? Real(-1) : Real(1);
    return Quaternion{p.w + same * q.w, p.x + same * q.x, p.y + same * q.y, p.z + same * q.z}.unit();
}

} // namespace

SoftContact::SoftContact(const ContactPair& pair, const std::vector<Body>& bodies, const StepArray<SolverBody>& states, const Softness& soft, Span<ContactForces::Entry> known, Real h, bool fourPoints,
                         bool tracked)
    : a_(pair.a), b_(pair.b), normal_(pair.manifold.normal), roundA_(pair.manifold.roundA), roundB_(pair.manifold.roundB), faceOnB_(pair.manifold.faceOnB), soft_(soft) {
    const Body &bodyA = bodies[size_t(pair.a)], &bodyB = bodies[size_t(pair.b)];
    const SolverBody &a = states[size_t(pair.a)], &b = states[size_t(pair.b)];
    if (tracked && ((roundA_ > 0 && roundB_ > 0) || (roundA_ + roundB_ > 0 && pair.manifold.count == 1))) shapes_.track(bodyA.part(pair.partA), bodyB.part(pair.partB));
    const Vector3 aside = abs(normal_.x) < Real(0.57) ? Vector3{1, 0, 0} : Vector3{0, 1, 0};
    tangent1_ = normal_.cross(aside), tangent1_ = (1 / tangent1_.length()) * tangent1_, tangent2_ = normal_.cross(tangent1_);
    friction_.coefficient = sqrt(bodyA.friction * bodyB.friction), restitution_ = std::max(bodyA.restitution, bodyB.restitution);
    const CentreFriction::Carried rubbed = takePoints(pair, bodyA.position, bodyB.position, a, b, known, h, fourPoints);
    normal0_ = normal_, tangent10_ = tangent1_, tangent20_ = tangent2_, apart_ = bodyB.position - bodyA.position;
    createdA_ = {a.moved, a.turned}, createdB_ = {b.moved, b.turned};
    edges_ = points_.size() == 1 && roundA_ == 0 && roundB_ == 0 && (points_[0].key.id & kEdgePair) == kEdgePair;
    friction_.prepare(a, b, frame(), rubbed);
}

// Точки площадки с плечами от центров масс xA и xB, их массы и накопленные импульсы из known (берётся
// последняя запись с номером точки); центр трения — среднее точек с весом по зазору. Возвращает
// накопленный импульс трения пары, мир.
CentreFriction::Carried SoftContact::takePoints(const ContactPair& pair, Vector3 xA, Vector3 xB, const SolverBody& a, const SolverBody& b, Span<ContactForces::Entry> known, Real h, bool fourPoints) {
    const Vector3 n = normal_;
    Real weights = 0;
    CentreFriction::Carried rubbed;
    StepArray<int> kept(size_t(pair.manifold.count));
    if (fourPoints && pair.manifold.count > 4) kept = fourOf(pair.manifold);
    else std::iota(kept.begin(), kept.end(), 0);
    points_.reserve(kept.size());
    for (int k : kept) {
        const ContactPoint& contact = pair.manifold.points[size_t(k)];
        Point p{};
        p.key = {pair.a, pair.b, contact.id, pair.partA, pair.partB};
        const Vector3 at = Real(0.5) * (contact.onA + contact.onB);
        p.rA = at - xA, p.rB = at - xB;
        p.coreA = p.rA - roundA_ * n, p.coreB = p.rB + roundB_ * n;
        p.base = p.gap = p.found = contact.separation;
        if (restitution_ != 0) {
            const Real response = SolverBody::inverseMassAlong(a, b, p.rA, p.rB, n);
            p.normalMass = response > 0 ? 1 / response : 0;
            p.approach = n.dot(b.v + b.w.cross(p.rB) - a.v - a.w.cross(p.rA));
        }
        size_t last = known.size();
        while (last > 0 && known[last - 1].first.id != contact.id) --last;
        if (last > 0) p.impulse = h * known[last - 1].second.normal, rubbed.rubbed += h * known[last - 1].second.tangent, rubbed.twist += h * known[last - 1].second.twist;
        const Real weight = std::max(kLeastWeight, std::min(Real(1), 2 - contact.separation / kSpeculative));
        centreA_ += weight * p.rA, centreB_ += weight * p.rB, weights += weight;
        points_.push_back(p);
    }
    centreA_ = (1 / weights) * centreA_, centreB_ = (1 / weights) * centreB_;
    centreCoreA_ = centreA_ - roundA_ * n, centreCoreB_ = centreB_ + roundB_ * n;
    for (Point& p : points_) p.lever = (p.rA - centreA_).length();
    return rubbed;
}


void SoftContact::aim(const BodyMotion& a, const BodyMotion& b, Vector3 apart, const ClosestPoints* found) {
    if (roundA_ > 0 && roundB_ > 0) {
        Vector3 between = apart + b.turned.rotate(points_[0].coreB) - a.turned.rotate(points_[0].coreA);
        if (found) between = found->normal;
        if (between.length() > 0) normal_ = (1 / between.length()) * between;
        tangent1_ = tangent10_;
    } else {
        const Quaternion turn = edges_ ? meanTurn(a.turned, b.turned) : faceOnB_ ? b.turned : a.turned;
        normal_ = turn.rotate(normal0_), tangent1_ = turn.rotate(tangent10_), tangent2_ = turn.rotate(tangent20_);
        if (!found) return;
        normal_ = found->normal;
    }
    tangent1_ = tangent1_ - tangent1_.dot(normal_) * normal_, tangent1_ = (1 / tangent1_.length()) * tangent1_, tangent2_ = normal_.cross(tangent1_);
}

void SoftContact::follow(const StepArray<SolverBody>& states) {
    const SolverBody &stateA = states[size_t(a_)], &stateB = states[size_t(b_)];
    const BodyMotion a{stateA.moved - createdA_.moved, stateA.turned * createdA_.turned.conjugate()}, b{stateB.moved - createdB_.moved, stateB.turned * createdB_.turned.conjugate()};
    const Vector3 apart = apart_ + b.moved - a.moved;
    // Нормаль круглого тела — по ближайшим точкам ядер. У шаров это линия центров. У капсулы и
    // цилиндра ближайшая точка ядра уходит вдоль него вместе с соседом: по точкам, закреплённым в
    // телах, нормаль наклонялась, а плечо отставало — шар, скользящий по торцу вращающегося цилиндра
    // со скоростью 4,7 м/с, входил в него на 12 мм. То же у круглого тела на ребре или вершине
    // многогранника, который вращается: нормаль, повёрнутая вместе с ним, уходит на 6–18° за
    // обновление. Поэтому ближайшие точки ищутся заново (GJK, TrackedShapes); у площадки из одной точки
    // по ним ставится и сама точка с её зазором.
    const ClosestPoints closest = shapes_.tracking() ? shapes_.closest(a, b, apart_) : ClosestPoints{};
    const bool found = shapes_.tracking() && !closest.coresOverlap;
    aim(a, b, apart, found ? &closest : nullptr);
    if (found && points_.size() == 1) {
        const Vector3 middle = Real(0.5) * (closest.onA + closest.onB);
        points_[0].rA = centreA_ = middle - a.moved, points_[0].rB = centreB_ = middle - (apart_ + b.moved);
        points_[0].gap = closest.distance;
        return;
    }
    // Плечи точек двух тел и расхождение этих точек d. Общая точка приложения — точка тела, которое
    // лежит на грани другого (она и есть место касания); у пары рёбер и у двух круглых тел — посередине.
    const Real toB = edges_ || (roundA_ > 0 && roundB_ > 0) ? Real(0.5) : faceOnB_ ? Real(0) : Real(1);
    const auto meet = [&](Vector3 coreA, Vector3 coreB, Vector3& rA, Vector3& rB) {
        rA = a.turned.rotate(coreA) + roundA_ * normal_, rB = b.turned.rotate(coreB) - roundB_ * normal_;
        const Vector3 d = apart + rB - rA;
        rA += toB * d, rB -= (1 - toB) * d;
        return d;
    };
    meet(centreCoreA_, centreCoreB_, centreA_, centreB_);
    for (Point& p : points_) p.gap = meet(p.coreA, p.coreB, p.rA, p.rB).dot(normal_) + p.base;
}

void SoftContact::warmStart(StepArray<SolverBody>& states) const {
    SolverBody &a = states[size_t(a_)], &b = states[size_t(b_)];
    for (const Point& p : points_) SolverBody::push(a, b, p.rA, p.rB, p.impulse * normal_);
    friction_.warmStart(a, b, frame());
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

void SoftContact::solveNormals(StepArray<SolverBody>& states, bool withBias, Real h, Real pushSpeed) {
    SolverBody &a = states[size_t(a_)], &b = states[size_t(b_)];
    const Vector3 n = normal_;
    const size_t count = points_.size();
    BlockWork& w = work;
    respond(a, b);
    for (size_t i = 0; i < count; ++i) {
        const Point& p = points_[i];
        const Real gap = p.gap;
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
    }
}

void SoftContact::setNormalImpulse(StepArray<SolverBody>& states, size_t k, Real impulse) {
    Point& p = points_[k];
    SolverBody::push(states[size_t(a_)], states[size_t(b_)], p.rA, p.rB, (impulse - p.impulse) * normal_);
    p.impulse = impulse;
}

void SoftContact::matchTrial(const StepArray<SolverBody>& states, Real h) {
    const SolverBody &a = states[size_t(a_)], &b = states[size_t(b_)];
    for (Point& p : points_) {
        const Real curved = p.found - (p.gap + h * normal_.dot(b.v + b.w.cross(p.rB) - a.v - a.w.cross(p.rA)));
        p.base += curved, p.gap += curved;
    }
}

void SoftContact::solveFriction(StepArray<SolverBody>& states, Real rollingArm) {
    Real load = 0, leverLoad = 0;
    for (const Point& p : points_) load += p.impulse, leverLoad += p.lever * p.impulse;
    friction_.solve(states[size_t(a_)], states[size_t(b_)], frame(), load, leverLoad, rollingArm);
}

// Удар становится свершившимся изменением скоростей: накопленный импульс точки обнуляется, иначе
// следующее решение нормалей (оно ищет полный импульс точки) сняло бы отскок как лишний.
void SoftContact::bounce(StepArray<SolverBody>& states, Real bounceSpeed) {
    if (restitution_ == 0) return;
    constexpr Real kTouching = Real(1e-3); // м: ближе этого упреждающая точка считается коснувшейся
    SolverBody &a = states[size_t(a_)], &b = states[size_t(b_)];
    for (Point& p : points_) {
        if (p.approach > -bounceSpeed || p.impulse <= 0 || p.gap > kTouching) continue;
        const Real speed = normal_.dot(b.v + b.w.cross(p.rB) - a.v - a.w.cross(p.rA));
        const Real next = std::max(p.impulse - p.normalMass * (speed + restitution_ * p.approach), Real(0));
        SolverBody::push(a, b, p.rA, p.rB, (next - p.impulse) * normal_);
        p.impulse = 0, p.approach = 0;
    }
}

void SoftContact::remember(ContactForces& memory, Real h) const {
    const Real share = 1 / (h * Real(points_.size()));
    const CentreFriction::Carried kept = friction_.carried(frame());
    for (const Point& p : points_) memory[p.key] = {p.impulse / h, share * kept.rubbed, share * kept.twist};
}

} // namespace kp
