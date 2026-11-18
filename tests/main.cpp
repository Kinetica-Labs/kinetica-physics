#include "Check.h"

void geometryTests();
void hullTests();
void collisionTests();
void dynamicsTests();

int main() {
    geometryTests();
    hullTests();
    collisionTests();
    dynamicsTests();
    std::printf("проверок %d, отказов %d\n", kp::test::checks, kp::test::failures);
    return kp::test::failures != 0;
}
