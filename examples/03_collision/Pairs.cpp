#include <kp/collision/PairFinder.h>
// Слой collision: какие тела касаются или успеют коснуться за шаг. Только эти пары идут дальше,
// в решатель; остальные тела друг о друге не знают.

#include <kp/collision/ContactPair.h>

#include <cstdio>

using namespace kp;

int main() {
    std::vector<Body> bodies = {
        Body::box({1, 0.5, 1}, 0, {0, -0.5, 0}),      // 0: пол
        Body::box({0.5, 0.5, 0.5}, 1000, {0, 0.5, 0}), // 1: куб лежит на полу
        Body::sphere(0.5, 1000, {0, 3, 0}),            // 2: шар падает на куб со скоростью 100 м/с
        Body::sphere(0.5, 1000, {50, 0, 0}),           // 3: шар далеко в стороне
    };
    bodies[2].linearMomentum = {0, -100 / bodies[2].mass.inverseMass, 0}; // импульс p = m v

    for (const ContactPair& pair : PairFinder(bodies, {0, -9.81, 0}).pairs(Real(1) / 60))
        std::printf("тела %d и %d: между ними %.3f м, за шаг могут сблизиться на %.3f м — %s\n", pair.a, pair.b, pair.distance,
                    pair.reach, pair.inContact() ? "касаются" : "ещё не касаются, нужен поиск времени удара");
    return 0;
}
