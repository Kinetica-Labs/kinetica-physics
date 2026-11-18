// Первый пример: куб падает на пол. Показывает весь путь — мир, тела, шаг, чтение результата.

#include <kp/world/World.h>

#include <cstdio>

using namespace kp;

int main() {
    World world; // сила тяжести по умолчанию: 9,81 м/с² вниз по оси Y

    world.addBody(Body::box({10, 0.5, 10}, 0, {0, -0.5, 0}));                 // пол: плотность 0 — неподвижное тело
    const BodyId cube = world.addBody(Body::box({0.5, 0.5, 0.5}, 1000, {0, 3, 0})); // куб со стороной 1 м на высоте 3 м

    for (int frame = 0; frame <= 120; ++frame) {
        if (frame % 10 == 0) std::printf("t = %.2f с   высота центра куба %.4f м\n", frame / 60.0, world.body(cube).position.y);
        world.step(1.0 / 60);
    }
    return 0; // куб лежит на полу: высота центра 0,5 м
}
