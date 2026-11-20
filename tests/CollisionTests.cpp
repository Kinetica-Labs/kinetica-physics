#include <collision/PairFinder.h>
// Проверки слоя collision: дерево поиска пар.

#include "Check.h"

#include <collision/ContactPair.h>
#include <collision/ManifoldMemory.h>

#include <algorithm>

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

// Для каждой точки площадки x — ближайшая к ней точка площадки y: наибольшее расстояние между ними и
// наибольшая разница их зазоров. Номера точек не сравниваются: у равных граней поиск берёт опорной
// то грань A, то грань B, и одна и та же точка получает разные номера.
struct Mismatch {
    Real place = 0, separation = 0;
};
Mismatch mismatchOf(const ContactManifold& x, const ContactManifold& y) {
    Mismatch worst;
    const auto middle = [](const ContactPoint& p) { return Real(0.5) * (p.onA + p.onB); };
    for (int i = 0; i < x.count; ++i) {
        int nearest = 0;
        for (int j = 1; j < y.count; ++j)
            if ((middle(x.points[size_t(i)]) - middle(y.points[size_t(j)])).length() < (middle(x.points[size_t(i)]) - middle(y.points[size_t(nearest)])).length()) nearest = j;
        worst.place = std::max(worst.place, (middle(x.points[size_t(i)]) - middle(y.points[size_t(nearest)])).length());
        worst.separation = std::max(worst.separation, Real(std::fabs(x.points[size_t(i)].separation - y.points[size_t(nearest)].separation)));
    }
    return worst;
}

// Площадка, которую память несёт за телами (ManifoldMemory), обязана совпасть со свежим поиском: при
// сдвиге пары в пределах допуска зазоры те же, а места точек расходятся не больше чем на сдвиг,
// делённый на синус угла между рёбрами (здесь не меньше 0,29 — допуск × 4). За допуском по сдвигу,
// по росту пути сближения или с другой формой запись не годится.
void carriedManifoldMatchesSearch() {
    std::uint64_t random = 11;
    const Vector3 gravity{0, -9.81, 0};
    const Real h = Real(1) / 240, tolerance = Real(2e-4);
    const auto within = [&](double size) { return Vector3{Real(size * (2 * uniform(random) - 1)), Real(size * (2 * uniform(random) - 1)), Real(size * (2 * uniform(random) - 1))}; };
    int carriedPairs = 0;
    for (int trial = 0; trial < 200; ++trial) {
        // Куб лежит на другом со сдвигом и поворотом вокруг вертикали: в площадке и вершины, и пересечения рёбер.
        const double angle = 0.3 + 0.9 * uniform(random);
        std::vector<Body> bodies = {Body::box({0.2, 0.2, 0.2}, 1000, {0, 0.2, 0}), Body::box({0.2, 0.2, 0.2}, 1000, {Real(0.25 * uniform(random)), Real(0.5999), Real(0.25 * uniform(random))})};
        bodies[1].orientation = Quaternion{Real(std::cos(angle / 2)), 0, Real(std::sin(angle / 2)), 0};
        const std::optional<ContactPair> found = examinePair(bodies, 0, 1, gravity, h);
        const std::optional<ManifoldMemory::Entry> entry = found ? ManifoldMemory::entryOf(*found, bodies[0], bodies[1]) : std::nullopt;
        if (!entry) continue;
        ManifoldMemory memory;
        memory.tolerance = tolerance;
        memory.replace(StepArray<ManifoldMemory::Entry>(1, *entry));
        const Vector3 turn = within(4e-5);
        bodies[1].position += within(4e-5), bodies[1].orientation = (Quaternion{1, turn.x, turn.y, turn.z} * bodies[1].orientation).unit();
        const ManifoldMemory::Entry* known = memory.valid(0, 1, bodies[0], bodies[1], found->reach);
        CHECK(known != nullptr, "проба %d: запись не годится при сдвиге в пределах допуска", trial);
        const std::optional<ContactPair> fresh = examinePair(bodies, 0, 1, gravity, h);
        if (!known || !fresh) continue;
        const Mismatch worst = mismatchOf(ManifoldMemory::carried(*known, bodies[0], bodies[1], found->reach).manifold, fresh->manifold);
        CHECK(fresh->manifold.count >= 2 && worst.separation < Real(1e-6) && worst.place < 4 * tolerance, "проба %d: зазоры расходятся на %.2e м, места точек на %.2e м", trial,
              double(worst.separation), double(worst.place));
        CHECK(memory.valid(0, 1, bodies[0], bodies[1], found->reach + 2 * tolerance) == nullptr, "проба %d: запись годится при выросшем пути сближения", trial);
        CHECK(memory.valid(0, 1, bodies[0], Body::box({0.2, 0.2, 0.2}, 1000, bodies[1].position), found->reach) == nullptr, "проба %d: запись досталась телу с другой формой", trial);
        bodies[1].position.x += 2 * tolerance;
        CHECK(memory.valid(0, 1, bodies[0], bodies[1], found->reach) == nullptr, "проба %d: запись годится при сдвиге за допуском", trial);
        ++carriedPairs;
    }
    CHECK(carriedPairs > 150, "площадок по грани слишком мало: %d из 200", carriedPairs);
}

} // namespace

void collisionTests() {
    carriedManifoldMatchesSearch();
    treeFindsEveryPair();
    treeStaysBalanced();
}
