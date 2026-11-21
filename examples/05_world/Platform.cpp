// Тело, движимое извне: плита без массы с заданной скоростью. Силы и удары её не меняют, а ящик стоит
// на ней как на опоре, которая движется. Так же задают двери, конвейеры и захват робота, ведомый программой.

#include <world/World.h>

#include <cstdio>

using namespace kp;

int main() {
    World world;
    Body plate = Body::box({0.5, 0.05, 0.5}, 0, {0, 0.5, 0}); // плотность 0: тело без массы
    plate.drivenVelocity = {0.5, 0, 0};                       // идёт вбок на 0,5 м/с, что бы на ней ни лежало
    const BodyId lift = world.addBody(plate);
    const BodyId crate = world.addBody(Body::box({0.2, 0.2, 0.2}, 1000, {0, 0.75, 0}));

    for (int frame = 1; frame <= 120; ++frame) {
        if (frame == 61) world.body(lift).drivenVelocity = {0, 0.5, 0}; // через секунду плита идёт вверх
        world.step(1.0 / 60);
        if (frame % 30 == 0) {
            const Vector3 p = world.body(lift).position, c = world.body(crate).position;
            std::printf("t = %.1f с   плита (%.3f; %.3f)   ящик отстал на %.4f м и стоит над плитой на %.4f м\n", frame / 60.0, p.x, p.y, p.x - c.x, c.y - p.y);
        }
    }
    return 0; // на рывке вбок ящик отстал на v² / 2 μ g = 2,5 см; рывок вверх — удар: его трение сняло половину
              // скорости ящика, и вперёд он проехал только 6 мм
}
