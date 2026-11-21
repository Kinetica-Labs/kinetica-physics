// Взрыв: импульс всем телам в шаре действия, вдоль луча от центра; чем дальше тело, тем меньше ему
// достаётся. Тело, задетое не по центру, закручивается.

#include <physics.h>

#include <cstdio>

using namespace kp;

int main() {
    World world;
    world.addBody(Body::box({10, 0.5, 10}, 0, {0, -0.5, 0}));
    for (int k = 0; k < 5; ++k) world.addBody(Body::box({0.1, 0.1, 0.1}, 1000, {Real(0.5 + 0.5 * k), 0.1, 0}));

    // Центр, радиус действия 2 м, импульс 40 Н·с у самого центра, линейный спад к границе шара.
    const int touched = applyRadialImpulse(world, {0, 0.1, 0}, 2, 40);
    std::printf("взрыв задел тел: %d из 5\n", touched);
    for (int k = 1; k <= 5; ++k) std::printf("куб в %.1f м от центра получил скорость %.3f м/с\n", 0.5 * k, world.body(BodyId(k)).linearVelocity().x);

    for (int frame = 0; frame < 120; ++frame) world.step(1.0 / 60);
    for (int k = 1; k <= 5; ++k) std::printf("через 2 с куб %d остановился в x = %.3f м\n", k, world.body(BodyId(k)).position.x);
    return 0;
}
