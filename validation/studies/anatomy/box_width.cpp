// Linked beside either unchanged Box3D benchmark adapter. Box3D's C functions below are
// internal to the pinned static library. World creation reads this setting once per world.
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" int b3GetSIMDWidth(void);
extern "C" void b3SetSIMDWidth(int width);

namespace {
struct WidthSelection {
    WidthSelection() {
        const char* force = std::getenv("KP_BOX_FORCE4");
        if (force && std::strcmp(force, "1") == 0) b3SetSIMDWidth(4);
        std::fprintf(stderr, "BOX_WIDTH=%d\n", b3GetSIMDWidth());
    }
};
const WidthSelection selection;
}
