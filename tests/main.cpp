#include "Check.h"

void geometryTests();
void hullTests();
void collisionTests();
void contactPatchTests();
void dynamicsTests();
void frictionTests();
void gearTests();
void linkageTests();
void wheelTests();
void vehicleTests();
void propertyTests();

int main() {
    geometryTests();
    hullTests();
    collisionTests();
    contactPatchTests();
    dynamicsTests();
    frictionTests();
    gearTests();
    linkageTests();
    wheelTests();
    vehicleTests();
    propertyTests();
    std::printf("проверок %d, отказов %d\n", kp::test::checks, kp::test::failures);
    return kp::test::failures != 0;
}
