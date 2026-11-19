#include "Check.h"

void geometryTests();
void hullTests();
void collisionTests();
void dynamicsTests();
void propertyTests();

int main() {
    geometryTests();
    hullTests();
    collisionTests();
    dynamicsTests();
    propertyTests();
    std::printf("проверок %d, отказов %d\n", kp::test::checks, kp::test::failures);
    return kp::test::failures != 0;
}
