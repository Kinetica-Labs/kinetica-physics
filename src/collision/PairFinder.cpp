#include <collision/PairFinder.h>

#include <math/WorkerPool.h>

#include <algorithm>

namespace kp {
namespace {

// Брусок вокруг тела: точные габариты формы вдоль осей мира, увеличенные на путь, который любая
// точка тела может пройти за h, и на полосу касания. Если бруски двух тел не пересеклись, тела за
// это время не сблизятся до касания, и examinePair такую пару отверг бы.
Bounds reachBounds(const Body& body, Vector3 gravity, Real h) {
    // Габарит вдоль оси — опорная точка формы в этом направлении плюс радиус скругления.
    const auto extent = [&](Vector3 axis) {
        const Vector3 local = body.pose().localDirection(axis);
        return local.dot(body.shape->support(local)) + body.shape->radius;
    };
    const Real reach = (maxPointSpeed(body) + gravity.length() * h) * h + kContactDistance;
    const Vector3 grow{reach, reach, reach};
    return {body.position - Vector3{extent({-1, 0, 0}), extent({0, -1, 0}), extent({0, 0, -1})} - grow,
            body.position + Vector3{extent({1, 0, 0}), extent({0, 1, 0}), extent({0, 0, 1})} + grow};
}

} // namespace

DynamicTree PairFinder::tree(Real h) const {
    DynamicTree tree;
    for (int i = 0; i < int(bodies_.size()); ++i)
        if (bodies_[size_t(i)].shape) tree.insert(reachBounds(bodies_[size_t(i)], gravity_, h), i);
    return tree;
}

namespace {

// Пары частей двух тел, из которых хотя бы одно составное: каждая часть одного с каждой частью
// другого. Если у одного из тел есть дерево частей, кандидатов отбирает оно; проверяет их тот же
// examinePair, и пары идут в том же порядке, поэтому ответ тот же, что у перебора.
std::vector<std::pair<int, int>> partPairs(const Body& a, const Body& b, Vector3 gravity, Real h) {
    std::vector<std::pair<int, int>> together;
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

// Пары частей тел i и j — в порядке partPairs.
std::vector<ContactPair> PairFinder::between(int i, int j, Real h) const {
    std::vector<ContactPair> pairs;
    const Body &a = bodies_[size_t(i)], &b = bodies_[size_t(j)];
    if (!a.parts && !b.parts) {
        if (const std::optional<ContactPair> pair = examinePair(bodies_, i, j, gravity_, h, boundary)) pairs.push_back(*pair);
        return pairs;
    }
    for (const auto& [p, q] : partPairs(a, b, gravity_, h)) {
        std::optional<ContactPair> pair = examinePair(a.part(p), b.part(q), gravity_, h, boundary);
        if (!pair) continue;
        pair->a = i, pair->b = j, pair->partA = p, pair->partB = q;
        pairs.push_back(*pair);
    }
    return pairs;
}

// Кандидатов отбирает дерево, в одном потоке; проверка каждой пары тел — отдельное задание пула
// (WorkerPool). Ответы склеиваются в порядке кандидатов, поэтому от числа потоков список не зависит.
// Сцену меньше kParallelParts частей дешевле проверить в одном потоке, чем будить остальные.
std::vector<ContactPair> PairFinder::pairs(Real h) const {
    constexpr int kParallelParts = 64;
    const DynamicTree tree = this->tree(h);
    std::vector<std::pair<int, int>> candidates;
    int parts = 0;
    for (int i = 0; i < int(bodies_.size()); ++i) {
        if (!bodies_[size_t(i)].shape) continue;
        parts += bodies_[size_t(i)].partCount();
        const size_t first = candidates.size();
        tree.query(reachBounds(bodies_[size_t(i)], gravity_, h), [&](int j) {
            if (j > i) candidates.push_back({i, j});
        });
        // Кандидаты — по возрастанию номера: порядок пар не должен зависеть от формы дерева.
        std::sort(candidates.begin() + std::ptrdiff_t(first), candidates.end());
    }
    std::vector<std::vector<ContactPair>> found(candidates.size());
    const auto examine = [&](size_t k) { found[k] = between(candidates[k].first, candidates[k].second, h); };
    if (parts < kParallelParts) for (size_t k = 0; k < candidates.size(); ++k) examine(k);
    else WorkerPool::shared().run(candidates.size(), examine);
    std::vector<ContactPair> pairs;
    for (const std::vector<ContactPair>& some : found) pairs.insert(pairs.end(), some.begin(), some.end());
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
