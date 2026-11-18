#include <kp/collision/PairFinder.h>
// Проверки слоя collision: дерево поиска пар.

#include "Check.h"

#include <kp/collision/ContactPair.h>

#include <cmath>
#include <cstdint>

using namespace kp;

namespace {

double uniform(std::uint64_t& state) { // детерминированный генератор, [0, 1)
    state = state * 6364136223846793005ull + 1442695040888963407ull;
    return double(state >> 11) / double(1ull << 53);
}

// Дерево обязано давать тот же список пар, что перебор всех пар: оно лишь отбрасывает тела,
// которые заведомо далеко. Проверяется на облаках движущихся кубов и шаров разной плотности.
void treeFindsEveryPair() {
    std::uint64_t random = 7;
    const Vector3 gravity{0, -9.81, 0};
    int pairs = 0;
    for (int scene = 0; scene < 40; ++scene) {
        const int count = 2 + scene * 5;
        const double side = 0.3 + 0.05 * scene;
        std::vector<Body> bodies = {Body::box({50, 0.5, 50}, 0, {0, -0.5, 0})};
        for (int k = 0; k < count; ++k) {
            const Real half = Real(0.05 + 0.1 * uniform(random));
            Body body = k % 2 ? Body::box({half, half, half}, 1000) : Body::sphere(half, 1000);
            body.position = {side * uniform(random), half + side * uniform(random), side * uniform(random)};
            body.linearMomentum = (1 / body.mass.inverseMass) * Vector3{4 * uniform(random) - 2, 4 * uniform(random) - 2, 4 * uniform(random) - 2};
            bodies.push_back(body);
        }
        const std::vector<ContactPair> found = PairFinder(bodies, gravity).pairs(Real(1) / 60);
        size_t next = 0;
        bool same = true;
        for (int i = 0; i < int(bodies.size()); ++i)
            for (int j = i + 1; j < int(bodies.size()); ++j)
                if (examinePair(bodies, i, j, gravity, Real(1) / 60)) same = same && next < found.size() && found[next].a == i && found[next++].b == j;
        CHECK(same && next == found.size(), "сцена %d: дерево нашло %zu пар, перебор — другие", scene, found.size());
        pairs += int(found.size());
    }
    CHECK(pairs > 1000, "сцены слишком редкие: всего %d пар", pairs);
}

// Дерево выровнено: высота не больше 1,44 log₂ n + 2 даже при вставке тел по порядку вдоль прямой —
// худшем порядке для дерева без поворотов, где высота была бы равна числу тел.
void treeStaysBalanced() {
    for (int count : {10, 100, 1000, 10000}) {
        DynamicTree tree;
        for (int k = 0; k < count; ++k) tree.insert({{Real(k), 0, 0}, {Real(k) + Real(0.5), 1, 1}}, k);
        CHECK(tree.height() <= int(1.44 * std::log2(double(count)) + 2), "высота %d при %d листьях", tree.height(), count);
        int visited = 0;
        tree.query({{Real(-1), -1, -1}, {Real(count), 2, 2}}, [&](int) { ++visited; });
        CHECK(visited == count, "запрос по всему объёму вернул %d листьев из %d", visited, count);
    }
}

} // namespace

void collisionTests() {
    treeFindsEveryPair();
    treeStaysBalanced();
}
