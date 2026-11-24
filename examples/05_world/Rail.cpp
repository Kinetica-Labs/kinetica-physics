// Сочленение по траектории: вагонетка идёт по замкнутой кривой, проведённой через точки. Привод держит
// скорость вдоль пути, вагонетка поворачивается вслед за кривой.

#include <world/World.h>

#include <cstdio>

using namespace kp;

int main() {
    World world;
    const BodyId ground = world.addBody(Body::box({3, 0.1, 3}, 0, {0, -0.1, 0}));
    const BodyId cart = world.addBody(Body::box({0.1, 0.05, 0.15}, 1000, {1, 0.5, 0}));

    // Кривая — квадрат со стороной 2 м на высоте 0,5 м; сплайн скругляет его углы.
    const std::vector<Vector3> through = {{1, 0.5, 0}, {1, 0.5, 1}, {0, 0.5, 1}, {-1, 0.5, 1}, {-1, 0.5, 0}, {-1, 0.5, -1}, {0, 0.5, -1}, {1, 0.5, -1}};
    PathJoint& rail = addPathJoint(world, ground, cart, {1, 0.5, 0}, through, true);
    rail.rotation = PathRotation::FollowPath;
    rail.holdSpeed = true, rail.driveSpeed = 1; // 1 м/с вдоль пути

    std::printf("длина пути %.3f м\n", rail.path().length());
    for (int frame = 1; frame <= 240; ++frame) {
        world.step(1.0 / 60);
        if (frame % 60 == 0) {
            const Vector3 p = world.body(cart).position;
            std::printf("t = %d с   вагонетка в (%.3f; %.3f; %.3f), прошла %.3f м\n", frame / 60, p.x, p.y, p.z, coordinate(world, rail));
        }
    }
    return 0;
}
