// Мяч с коэффициентом восстановления e отскакивает на высоту e² от предыдущей.
// Удар решается в тот момент, когда он произошёл, поэтому высоты совпадают с формулой.

#include <kp/world/World.h>

#include <cstdio>

using namespace kp;

int main() {
    const Real radius = 0.1, restitution = 0.8;
    World world;

    Body floor = Body::box({10, 0.5, 10}, 0, {0, -0.5, 0});
    Body ball = Body::sphere(radius, 1000, {0, radius + 1, 0}); // низ мяча на высоте 1 м
    floor.restitution = ball.restitution = restitution;
    world.addBody(floor);
    const BodyId id = world.addBody(ball);

    Real expected = 1, previousSpeed = 0;
    for (int frame = 0; frame < 300; ++frame) {
        world.step(1.0 / 60);
        const Real speed = world.body(id).linearVelocity().y;
        if (previousSpeed > 0 && speed <= 0) { // вершина отскока: подъём сменился падением
            expected *= restitution * restitution;
            std::printf("отскок до %.4f м, по формуле %.4f м\n", world.body(id).position.y - radius, expected);
        }
        previousSpeed = speed;
    }
    return 0;
}
