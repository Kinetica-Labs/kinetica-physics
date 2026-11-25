#include <math/StepMemory.h>
#include <collision/PairFinder.h>

#include <math/WorkerPool.h>

#include <algorithm>

namespace kp {
namespace {

// Брусок вокруг тела: точные габариты формы вдоль осей мира, увеличенные на путь, который любая
// точка тела может пройти за h, и на полосу касания. Если бруски двух тел не пересеклись, тела за
// это время не сблизятся до касания, и examinePair такую пару отверг бы. fastest — maxPointSpeed тела.
Bounds reachBounds(const Body& body, Real fastest, Vector3 gravity, Real h, Real slack) {
    // Габарит вдоль оси — опорная точка формы в этом направлении плюс радиус скругления.
    const auto extent = [&](Vector3 axis) {
        const Vector3 local = body.pose().localDirection(axis);
        return local.dot(body.shape->support(local)) + body.shape->radius;
    };
    const Real reach = (fastest + gravity.length() * h) * h + kContactDistance + slack;
    const Vector3 grow{reach, reach, reach};
    return {body.position - Vector3{extent({-1, 0, 0}), extent({0, -1, 0}), extent({0, 0, -1})} - grow,
            body.position + Vector3{extent({1, 0, 0}), extent({0, 1, 0}), extent({0, 0, 1})} + grow};
}

// Наибольшая скорость точки каждого тела за время h: нынешняя (maxPointSpeed) и прибавка от приложенных
// к телу силы и момента (forcedSpeedGain).
StepArray<Real> speedsOf(const std::vector<Body>& bodies, Real h) {
    StepArray<Real> fastest(bodies.size());
    for (size_t i = 0; i < bodies.size(); ++i) fastest[i] = maxPointSpeed(bodies[i]) + forcedSpeedGain(bodies[i], h);
    return fastest;
}

// Лист дерева охватывает досягаемость тела за h. Тела без формы пропускаются.
DynamicTree reachTree(const std::vector<Body>& bodies, const StepArray<Real>& fastest, Vector3 gravity, Real h, Real slack) {
    DynamicTree tree;
    tree.reserve(int(bodies.size()));
    for (size_t i = 0; i < bodies.size(); ++i)
        if (bodies[i].shape) tree.insert(reachBounds(bodies[i], fastest[i], gravity, h, slack), int(i));
    return tree;
}

} // namespace

DynamicTree PairFinder::tree(Real h) const { return reachTree(bodies_, speedsOf(bodies_, h), gravity_, h, 0); }

// Кандидаты — пары тел, чьи бруски досягаемости за время h пересеклись; отбирает их дерево брусков.
// Идут по возрастанию номеров: порядок пар не должен зависеть от формы дерева. Пары тел, которые друг
// друга не касаются (mayTouch), отброшены и здесь, и из памяти кандидатов: у тела мог смениться фильтр.
StepArray<std::pair<int, int>> PairFinder::candidates(Real h, const StepArray<Real>& fastest) const {
    const auto touching = [&](StepArray<std::pair<int, int>> pairs) {
        pairs.erase(std::remove_if(pairs.begin(), pairs.end(), [&](const std::pair<int, int>& p) {
            return !mayTouch(bodies_[size_t(p.first)], bodies_[size_t(p.second)]);
        }), pairs.end());
        return pairs;
    };
    StepArray<Real> path;
    if (remembered) {
        path.resize(bodies_.size());
        const Real gravitySpeed = gravity_.length() * h;
        for (size_t i = 0; i < path.size(); ++i) path[i] = (fastest[i] + gravitySpeed) * h + kContactDistance;
        if (const auto* kept = remembered->valid(bodies_, path)) return touching({kept->begin(), kept->end()});
    }
    const DynamicTree tree = reachTree(bodies_, fastest, gravity_, h, remembered ? CandidateMemory::kSlack : Real(0));
    StepArray<std::pair<int, int>> candidates;
    tree.queryPairs([&](int a, int b) { candidates.emplace_back(std::min(a, b), std::max(a, b)); });
    std::sort(candidates.begin(), candidates.end());
    if (remembered) remembered->replace(bodies_, path, candidates);
    return touching(std::move(candidates));
}

namespace {

// Пары частей двух тел, из которых хотя бы одно составное: каждая часть одного с каждой частью
// другого. Если у одного из тел есть дерево частей, кандидатов отбирает оно; проверяет их тот же
// examinePair, и пары идут в том же порядке, поэтому ответ тот же, что у перебора.
StepArray<std::pair<int, int>> partPairs(const Body& a, const Body& b, Vector3 gravity, Real h) {
    StepArray<std::pair<int, int>> together;
    const Body& treed = a.partTree ? a : b;
    if (treed.partTree) {
        const Body& other = a.partTree ? b : a;
        // Наибольший путь сближения, какой examinePair насчитает паре частей этих тел.
        const Vector3 inverse = treed.mass.inverseInertia;
        const Real fastest = treed.linearVelocity().length() + treed.angularMomentum.length() * std::max({inverse.x, inverse.y, inverse.z}) * treed.shape->boundingRadius() + forcedSpeedGain(treed, h);
        for (int q = 0; q < other.partCount(); ++q) {
            const Body part = other.part(q);
            const Vector3 centre = frameOf(treed).localPoint(part.pose().position);
            const Real reach = part.shape->boundingRadius() + (fastest + maxPointSpeed(part) + forcedSpeedGain(part, h) + gravity.length() * h) * h + kContactDistance;
            const Vector3 grow{reach, reach, reach};
            treed.partTree->query({centre - grow, centre + grow}, [&](int p) { together.push_back(a.partTree ? std::make_pair(p, q) : std::make_pair(q, p)); });
        }
        std::sort(together.begin(), together.end());
    } else {
        for (int p = 0; p < a.partCount(); ++p)
            for (int q = 0; q < b.partCount(); ++q) together.push_back({p, q});
    }
    return together;
}

} // namespace

bool PairFinder::listed(int i, int j, int partA, int partB) const { return known && std::binary_search(known->begin(), known->end(), Key{i, j, partA, partB}); }

// Пара частей: площадка из памяти, если запись пары ещё годится, иначе поиск. a и b — части (Body::part).
void PairFinder::part(const Body& a, const Body& b, const Key& key, Real reach, Found& found) const {
    if (listed(key[0], key[1], key[2], key[3])) return;
    const ManifoldMemory::Entry* carried = memory ? memory->valid(key, a, b, reach) : nullptr;
    if (carried) found.pairs.push_back({ManifoldMemory::carried(*carried, a, b, reach), carried, -1});
    else if (const std::optional<ContactPair> pair = examinePairWithin(a, b, reach, boundary)) found.pairs.push_back({*pair, nullptr, -1});
    else return;
    Found::Pair& last = found.pairs.back();
    last.pair.a = key[0], last.pair.b = key[1], last.pair.partA = key[2], last.pair.partB = key[3];
    if (carried || !memory) return;
    if (const std::optional<ManifoldMemory::Entry> fresh = ManifoldMemory::entryOf(last.pair, a, b)) last.fresh = std::ptrdiff_t(found.fresh.size()), found.fresh.push_back(*fresh);
}

// Пары частей тел i и j — в порядке partPairs. reach — путь сближения простых тел (approachReach); у пары
// частей он свой, по границам скорости точек этих частей.
PairFinder::Found PairFinder::between(int i, int j, Real h, Real reach) const {
    const Body &a = bodies_[size_t(i)], &b = bodies_[size_t(j)];
    Found found;
    if (!a.parts && !b.parts) return part(a, b, {i, j, 0, 0}, reach, found), found;
    for (const auto& [p, q] : partPairs(a, b, gravity_, h)) {
        const Body partA = a.part(p), partB = b.part(q);
        if (!partA.shape || !partB.shape || (partA.isStatic() && partB.isStatic())) continue;
        part(partA, partB, {i, j, p, q}, approachReach(maxPointSpeed(partA) + forcedSpeedGain(partA, h), maxPointSpeed(partB) + forcedSpeedGain(partB, h), gravity_, h), found);
    }
    return found;
}

// Наибольшая скорость точки считается раз на тело, а не на каждую его пару. Кандидатов отбирает
// дерево, в одном потоке; проверка каждой пары тел — отдельное задание пула (WorkerPool). Ответы
// склеиваются в порядке кандидатов, поэтому от числа потоков список не зависит. Сцену меньше
// kParallelParts частей дешевле проверить в одном потоке, чем будить остальные.
std::vector<ContactPair> PairFinder::pairs(Real h) const {
    std::vector<ContactPair> pairs; // ответ уходит наружу и живёт дольше шага: он в куче, одним куском
    collect(h, pairs);
    return pairs;
}

// Для прежних площадок простых тел сразу заполняем выход. При промахе общий поиск
// заново собирает весь список, поэтому порядок, отбор частей и срок жизни записей те же.
bool PairFinder::carryAll(Real h, const StepArray<Real>& fastest, const StepArray<std::pair<int, int>>& candidates, std::vector<ContactPair>& pairs) const {
    if (!memory || known) return false;
    StepArray<const ManifoldMemory::Entry*> kept;
    kept.reserve(candidates.size());
    pairs.clear();
    for (const auto& [i, j] : candidates) {
        const Body &a = bodies_[size_t(i)], &b = bodies_[size_t(j)];
        if (a.parts || b.parts) return false;
        const Real reach = approachReach(fastest[size_t(i)], fastest[size_t(j)], gravity_, h);
        const auto* entry = memory->valid({i, j, 0, 0}, a, b, reach);
        if (!entry) return false;
        auto& pair = pairs.emplace_back();
        ManifoldMemory::carryInto(*entry, a, b, reach, pair);
        pair.a = i, pair.b = j;
        kept.push_back(entry);
    }
    memory->replace(kept);
    return true;
}

void PairFinder::collect(Real h, std::vector<ContactPair>& pairs) const {
    constexpr int kParallelParts = 64;
    const StepArray<Real> fastest = speedsOf(bodies_, h);
    int parts = 0;
    for (const Body& body : bodies_) parts += body.shape ? body.partCount() : 0;
    const StepArray<std::pair<int, int>> candidates = this->candidates(h, fastest);
    if (parts < kParallelParts && carryAll(h, fastest, candidates, pairs)) return;
    StepArray<Found> found(candidates.size());
    const auto examine = [&](size_t k) {
        const auto [i, j] = candidates[k];
        found[k] = between(i, j, h, approachReach(fastest[size_t(i)], fastest[size_t(j)], gravity_, h));
    };
    if (parts < kParallelParts) for (size_t k = 0; k < candidates.size(); ++k) examine(k);
    else WorkerPool::shared().run(candidates.size(), examine);
    size_t total = 0;
    StepArray<const ManifoldMemory::Entry*> kept; // кандидаты идут по возрастанию тел, пары частей внутри — тоже
    kept.reserve(memory ? found.size() : 0);
    for (const Found& some : found) {
        total += some.pairs.size();
        for (const Found::Pair& one : some.pairs)
            if (one.carried || one.fresh >= 0) kept.push_back(one.carried ? one.carried : &some.fresh[size_t(one.fresh)]);
    }
    if (memory) memory->replace(kept);
    pairs.clear(), pairs.reserve(total);
    for (const Found& some : found)
        for (const Found::Pair& one : some.pairs) pairs.push_back(one.pair);
}

std::vector<ContactPair> PairFinder::pairs(Real h, const JointSet& joints) const {
    std::vector<ContactPair> pairs;
    collect(h, joints, pairs);
    return pairs;
}

void PairFinder::collect(Real h, const JointSet& joints, std::vector<ContactPair>& pairs) const {
    collect(h, pairs);
    const auto jointed = [&](const ContactPair& pair) {
        for (const Joint& joint : joints)
            if (!joint.collides() && ((joint.a == pair.a && joint.b == pair.b) || (joint.a == pair.b && joint.b == pair.a))) return true;
        return false;
    };
    pairs.erase(std::remove_if(pairs.begin(), pairs.end(), jointed), pairs.end());
}

} // namespace kp
