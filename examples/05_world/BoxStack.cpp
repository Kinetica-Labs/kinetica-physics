// Стопка из десяти кубов стоит минуту и не сползает: силы контакта находятся точно,
// для всей стопки разом.

#include <world/World.h>

#include <cstdio>

using namespace kp;

int main() {
    const int count = 10;
    const Real half = 0.1;
    World world;
    world.addBody(Body::box({10, 0.5, 10}, 0, {0, -0.5, 0}));
    BodyId top{};
    for (int k = 0; k < count; ++k) top = world.addBody(Body::box({half, half, half}, 1000, {0, half + 2 * half * k, 0}));

    for (int frame = 0; frame < 60 * 60; ++frame) world.step(1.0 / 60);

    const Body& cube = world.body(top);
    std::printf("верхний куб через минуту: высота %.5f м (в начале %.5f), сдвиг вбок %.2e м, скорость %.2e м/с\n",
                cube.position.y, half + 2 * half * (count - 1), (Vector3{cube.position.x, 0, cube.position.z}).length(),
                cube.linearVelocity().length());
    return 0;
}
