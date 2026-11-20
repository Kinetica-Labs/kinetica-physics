#include <math/StepMemory.h>
#include <collision/PairFinder.h>

#include <math/WorkerPool.h>

#include <algorithm>

namespace kp {
namespace {

// Брусок вокруг тела: точные габариты формы вдоль осей мира, увеличенные на путь, который любая
// точка тела может пройти за h, и на полосу касания. Если бруски двух тел не пересеклись, тела за
// это время не сблизятся до касания, и examinePair такую пару отверг бы. fastest — maxPointSpeed тела.
Bounds reachBounds(const Body& body, Real fastest, Vector3 gravity, Real h) {
    // Габарит вдоль оси — опорная точка формы в этом направлении плюс радиус скругления.
    const auto extent = [&](Vector3 axis) {
        const Vector3 local = body.pose().localDirection(axis);
        return local.dot(body.shape->support(local)) + body.shape->radius;
    };
    const Real reach = (fastest + gravity.length() * h) * h + kContactDistance;
    const Vector3 grow{reach, reach, reach};
    return {body.position - Vector3{extent({-1, 0, 0}), extent({0, -1, 0}), extent({0, 0, -1})} - grow,
            body.position + Vector3{extent({1, 0, 0}), extent({0, 1, 0}), extent({0, 0, 1})} + grow};
}

// Наибольшая скорость точки каждого тела (maxPointSpeed).
StepArray<Real> speedsOf(const std::vector<Body>& bodies) {
    StepArray<Real> fastest(bodies.size());
    for (size_t i = 0; i < bodies.size(); ++i) fastest[i] = maxPointSpeed(bodies[i]);
    return fastest;
}

// Бруски досягаемости тел за время h и дерево из них; у тела без формы бруска нет.
struct Reaches {
    StepArray<Bounds> bounds;
    DynamicTree tree;

    Reaches(const std::vector<Body>& bodies, const StepArray<Real>& fastest, Vector3 gravity, Real h) : bounds(bodies.size()) {
        tree.reserve(int(bodies.size()));
        for (int i = 0; i < int(bodies.size()); ++i)
            if (bodies[size_t(i)].shape) tree.insert(bounds[size_t(i)] = reachBounds(bodies[size_t(i)], fastest[size_t(i)], gravity, h), i);
    }
};

} // namespace

DynamicTree PairFinder::tree(Real h) const { return Reaches(bodies_, speedsOf(bodies_), gravity_, h).tree; }

// Кандидаты — пары тел, чьи бруски досягаемости за время h пересеклись; отбирает их дерево брусков.
// Идут по возрастанию номеров: порядок пар не должен зависеть от формы дерева.
StepArray<std::pair<int, int>> PairFinder::candidates(Real h, const StepArray<Real>& fastest) const {
    const Reaches reach(bodies_, fastest, gravity_, h);
    StepArray<std::pair<int, int>> candidates;
    for (int i = 0; i < int(bodies_.size()); ++i) {
        if (!bodies_[size_t(i)].shape) continue;
        const size_t first = candidates.size();
        reach.tree.query(reach.bounds[size_t(i)], [&](int j) {
            if (j > i) candidates.push_back({i, j});
        });
        std::sort(candidates.begin() + std::ptrdiff_t(first), candidates.end());
    }
    return candidates;
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
        const Real fastest = treed.linearVelocity().length() + treed.angularMomentum.length() * std::max({inverse.x, inverse.y, inverse.z}) * treed.shape->boundingRadius();
        for (int q = 0; q < other.partCount(); ++q) {
            const Body part = other.part(q);
            const Vector3 centre = frameOf(treed).localPoint(part.pose().position);
            const Real reach = part.shape->boundingRadius() + (fastest + maxPointSpeed(part) + gravity.length() * h) * h + kContactDistance;
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

// Пара простых тел: площадка из памяти, если запись пары ещё годится, иначе поиск.
PairFinder::Found PairFinder::simplePair(int i, int j, Real reach) const {
    Found found;
    const Body &a = bodies_[size_t(i)], &b = bodies_[size_t(j)];
    found.carried = memory ? memory->valid(i, j, a, b, reach) : nullptr;
    if (found.carried) found.pairs.push_back(ManifoldMemory::carried(*found.carried, a, b, reach));
    else if (const std::optional<ContactPair> pair = examinePairWithin(a, b, reach, boundary)) found.pairs.push_back(*pair);
    if (found.pairs.empty()) return found;
    found.pairs[0].a = i, found.pairs[0].b = j;
    if (memory && !found.carried) found.fresh = ManifoldMemory::entryOf(found.pairs[0], a, b);
    return found;
}

// Пары частей тел i и j — в порядке partPairs. reach — путь сближения простых тел (approachReach).
PairFinder::Found PairFinder::between(int i, int j, Real h, Real reach) const {
    const Body &a = bodies_[size_t(i)], &b = bodies_[size_t(j)];
    if (!a.parts && !b.parts) return simplePair(i, j, reach);
    Found found;
    for (const auto& [p, q] : partPairs(a, b, gravity_, h)) {
        std::optional<ContactPair> pair = examinePair(a.part(p), b.part(q), gravity_, h, boundary);
        if (!pair) continue;
        pair->a = i, pair->b = j, pair->partA = p, pair->partB = q;
        found.pairs.push_back(*pair);
    }
    return found;
}

// Наибольшая скорость точки считается раз на тело, а не на каждую его пару. Кандидатов отбирает
// дерево, в одном потоке; проверка каждой пары тел — отдельное задание пула (WorkerPool). Ответы
// склеиваются в порядке кандидатов, поэтому от числа потоков список не зависит. Сцену меньше
// kParallelParts частей дешевле проверить в одном потоке, чем будить остальные.
std::vector<ContactPair> PairFinder::pairs(Real h) const {
    constexpr int kParallelParts = 64;
    const StepArray<Real> fastest = speedsOf(bodies_);
    int parts = 0;
    for (const Body& body : bodies_) parts += body.shape ? body.partCount() : 0;
    const StepArray<std::pair<int, int>> candidates = this->candidates(h, fastest);
    StepArray<Found> found(candidates.size());
    const auto examine = [&](size_t k) {
        const auto [i, j] = candidates[k];
        found[k] = between(i, j, h, approachReach(fastest[size_t(i)], fastest[size_t(j)], gravity_, h));
    };
    if (parts < kParallelParts) for (size_t k = 0; k < candidates.size(); ++k) examine(k);
    else WorkerPool::shared().run(candidates.size(), examine);
    size_t total = 0;
    StepArray<const ManifoldMemory::Entry*> kept;
    kept.reserve(memory ? found.size() : 0);
    for (const Found& some : found) {
        total += some.pairs.size();
        if (some.carried || some.fresh) kept.push_back(some.carried ? some.carried : &*some.fresh);
    }
    if (memory) memory->replace(kept);
    std::vector<ContactPair> pairs; // ответ уходит наружу и живёт дольше шага: он в куче, одним куском
    pairs.reserve(total);
    for (const Found& some : found) pairs.insert(pairs.end(), some.pairs.begin(), some.pairs.end());
    return pairs;
}

std::vector<ContactPair> PairFinder::pairs(Real h, const JointSet& joints) const {
    std::vector<ContactPair> pairs = this->pairs(h);
    const auto jointed = [&](const ContactPair& pair) {
        for (const Joint& joint : joints)
            if (!joint.collides() && ((joint.a == pair.a && joint.b == pair.b) || (joint.a == pair.b && joint.b == pair.a))) return true;
        return false;
    };
    pairs.erase(std::remove_if(pairs.begin(), pairs.end(), jointed), pairs.end());
    return pairs;
}

} // namespace kp
