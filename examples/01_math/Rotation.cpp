// Слой math: поворот вектора кватернионом и перевод точки из осей тела в мир.

#include <math/Transform.h>

#include <cstdio>

using namespace kp;

void print(const char* name, Vector3 v) { std::printf("%-28s (%.3f, %.3f, %.3f)\n", name, v.x, v.y, v.z); }

int main() {
    // Поворот на угол φ вокруг оси n — это кватернион (cos φ/2, n sin φ/2).
    // Для 90° вокруг Z обе половинки равны, поэтому хватает (1, 0, 0, 1) и нормировки.
    const Quaternion quarterTurn = Quaternion{1, 0, 0, 1}.unit();

    const Vector3 turned = quarterTurn.rotate({1, 0, 0});
    print("ось X после поворота:", turned);                             // (0, 1, 0)
    print("и обратно:", quarterTurn.rotateInverse(turned));            // (1, 0, 0)
    print("два поворота подряд:", (quarterTurn * quarterTurn).rotate({1, 0, 0})); // (−1, 0, 0)

    // Поза тела — где его центр и как оно повёрнуто. Точка, заданная в осях тела, переводится в мир.
    const Transform body{{10, 0, 0}, quarterTurn};
    print("угол тела (1, 0, 0) в мире:", body.point({1, 0, 0}));        // (10, 1, 0)
    return 0;
}
