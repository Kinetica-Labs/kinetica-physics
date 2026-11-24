#include "Check.h"

void geometryTests();
void hullTests();
void collisionTests();
void dynamicsTests();
void gearTests();
void linkageTests();
void wheelTests();
void vehicleTests();
void propertyTests();

int main() {
    geometryTests();
    hullTests();
    collisionTests();
    dynamicsTests();
    gearTests();
    linkageTests();
    wheelTests();
    vehicleTests();
    propertyTests();
    std::printf("проверок %d, отказов %d\n", kp::test::checks, kp::test::failures);
    return kp::test::failures != 0;
}
