// Маятник на шарнире: брусок качается вокруг оси на своём конце. Период малых колебаний
// физического маятника — T = 2π √(I / m g d): I — момент инерции вокруг оси, d — от оси до центра масс.

#include <world/World.h>

#include <cstdio>

using namespace kp;

int main() {
    const Real half = 0.5, thick = 0.02; // стержень длиной 1 м
    World world;
    const BodyId base = world.addBody(Body::box({0.05, 0.05, 0.05}, 0, {0, 2, 0})); // плотность 0 — неподвижная опора
    const BodyId bar = world.addBody(Body::box({thick, half, thick}, 1000, {0, 2 - half, 0}));
    HingeJoint& hinge = addHinge(world, base, bar, {0, 2, 0}, {0, 0, 1}); // общая точка и ось в мире
    world.body(bar).angularMomentum = {0, 0, Real(0.05) / world.body(bar).mass.inverseInertia.z}; // лёгкий толчок

    // Момент инерции вокруг оси на конце: свой плюс m d² (теорема Штейнера).
    const Real mass = 1 / world.body(bar).mass.inverseMass, inertia = 1 / world.body(bar).mass.inverseInertia.z + mass * half * half;
    const Real expected = 2 * kPi * kp::sqrt(inertia / (mass * Real(9.81) * half));

    Real previous = coordinate(world, hinge), crossed = -1;
    for (int frame = 1; frame <= 3000; ++frame) { // 5 с: три полных колебания
        world.step(1.0 / 600);
        const Real angle = coordinate(world, hinge);
        if (previous < 0 && angle >= 0) { // маятник прошёл низ в одну и ту же сторону
            const Real time = (frame - angle / (angle - previous)) / 600;
            if (crossed >= 0) std::printf("период %.5f с, по формуле %.5f с\n", time - crossed, expected);
            crossed = time;
        }
        previous = angle;
    }
    return 0;
}
