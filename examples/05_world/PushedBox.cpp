// Сила на теле: ящик толкают по полу с постоянной силой F. Трение μ m g тормозит, и ящик идёт с
// ускорением F / m − μ g. Сила действует один шаг мира, поэтому прикладывается перед каждым шагом.

#include <world/World.h>

#include <cstdio>

using namespace kp;

int main() {
    World world;
    world.addBody(Body::box({10, 0.5, 10}, 0, {0, -0.5, 0}));
    const BodyId crate = world.addBody(Body::box({0.2, 0.2, 0.2}, 1000, {0, 0.2, 0}));

    const Real mass = 1 / world.body(crate).mass.inverseMass, friction = 0.5; // трение пары: √(0,5 · 0,5)
    const Real force = 400;                                                     // Н, вдоль пола
    const Real acceleration = force / mass - friction * Real(9.81);

    for (int frame = 1; frame <= 120; ++frame) {
        applyForce(world.body(crate), {force, 0, 0}, world.body(crate).position); // в центр масс: ящик не закручивается
        world.step(1.0 / 60);
        if (frame % 30 == 0) {
            const Real time = frame / Real(60);
            std::printf("t = %.1f с   ящик прошёл %.4f м, по формуле %.4f м\n", time, world.body(crate).position.x, acceleration * time * time / 2);
        }
    }
    return 0;
}
