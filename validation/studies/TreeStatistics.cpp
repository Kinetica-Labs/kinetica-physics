// Прибор: статистика по дереву поиска пар. На случайных сценах сравнивает список пар, найденный
// через дерево, с перебором всех пар — они обязаны совпасть до последней пары — и печатает
// качество дерева: высоту против log₂ n и сколько кандидатов на тело оно оставляет.

#include <collision/PairFinder.h>
#include <collision/ContactPair.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>

using namespace kp;

namespace {

double uniform(std::uint64_t& state) { // детерминированный генератор, [0, 1)
    state = state * 6364136223846793005ull + 1442695040888963407ull;
    return double(state >> 11) / double(1ull << 53);
}

// Перебор всех пар — эталон, с которым сравнивается дерево.
std::vector<ContactPair> allPairs(const std::vector<Body>& bodies, Vector3 gravity, Real h) {
    std::vector<ContactPair> pairs;
    for (int i = 0; i < int(bodies.size()); ++i)
        for (int j = i + 1; j < int(bodies.size()); ++j)
            if (const std::optional<ContactPair> pair = examinePair(bodies, i, j, gravity, h)) pairs.push_back(*pair);
    return pairs;
}

template <class Run> double milliseconds(Run&& run) {
    const auto start = std::chrono::steady_clock::now();
    run();
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

// Сцены: облако — тела разбросаны в кубе; стопка — столбик вплотную; слой — плотная укладка на плите.
std::vector<Body> scene(const char* kind, int count, std::uint64_t& random) {
    std::vector<Body> bodies = {Body::box({1000, 0.5, 1000}, 0, {0, -0.5, 0})};
    const double side = std::cbrt(double(count)) * 0.5; // плотность облака: тело на 0,125 м³
    const int row = int(std::ceil(std::sqrt(double(count))));
    for (int k = 0; k < count; ++k) {
        const Real half = Real(0.05 + 0.1 * uniform(random));
        Body body = uniform(random) < 0.5 ? Body::box({half, half, half}, 1000) : Body::sphere(half, 1000);
        if (kind[0] == 'c') body.position = {side * uniform(random), half + side * uniform(random), side * uniform(random)};
        if (kind[0] == 's') body = Body::box({0.1, 0.1, 0.1}, 1000, {0, 0.1 + 0.2 * k, 0});
        if (kind[0] == 'l') body = Body::box({0.1, 0.1, 0.1}, 1000, {0.2 * (k % row), 0.1, 0.2 * (k / row)});
        body.linearMomentum = (1 / body.mass.inverseMass) * Vector3{2 * uniform(random) - 1, 2 * uniform(random) - 1, 2 * uniform(random) - 1};
        bodies.push_back(body);
    }
    return bodies;
}

} // namespace

int main() {
    const Vector3 gravity{0, -9.81, 0};
    const Real h = Real(1) / 60;
    std::uint64_t random = 2026;
    int scenes = 0, mismatches = 0;
    std::printf("%-8s %7s %7s %8s %10s %11s %11s %12s %9s\n", "сцена", "тел", "пар", "высота", "log2(n)", "кандидатов", "дерево, мс", "перебор, мс", "совпало");
    for (const char* kind : {"cloud", "stack", "layer"}) {
        for (int count : {10, 100, 1000, 4000}) {
            for (int repeat = 0; repeat < (count <= 100 ? 20 : 2); ++repeat) {
                const std::vector<Body> bodies = scene(kind, count, random);
                std::vector<ContactPair> found, reference;
                const double fast = milliseconds([&] { found = PairFinder(bodies, gravity).pairs(h); });
                const double slow = milliseconds([&] { reference = allPairs(bodies, gravity, h); });
                bool same = found.size() == reference.size();
                for (size_t p = 0; same && p < found.size(); ++p)
                    same = found[p].a == reference[p].a && found[p].b == reference[p].b && found[p].distance == reference[p].distance &&
                           found[p].manifold.count == reference[p].manifold.count;
                ++scenes;
                mismatches += !same;
                if (repeat > 0) continue;
                const DynamicTree tree = PairFinder(bodies, gravity).tree(h);
                long candidates = 0;
                for (int i = 0; i < int(bodies.size()); ++i) {
                    const Vector3 r{bodies[size_t(i)].shape->boundingRadius(), bodies[size_t(i)].shape->boundingRadius(), bodies[size_t(i)].shape->boundingRadius()};
                    tree.query({bodies[size_t(i)].position - r, bodies[size_t(i)].position + r}, [&](int j) { candidates += j > i; });
                }
                std::printf("%-8s %7d %7zu %8d %10.1f %11.1f %11.2f %12.2f %9s\n", kind, count, found.size(), tree.height(), std::log2(double(bodies.size())),
                            double(candidates) / bodies.size(), fast, slow, same ? "да" : "НЕТ");
            }
        }
    }
    std::printf("сцен %d, расхождений с перебором %d\n", scenes, mismatches);
    return mismatches != 0;
}
