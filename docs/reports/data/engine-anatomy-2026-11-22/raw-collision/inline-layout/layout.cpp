#include <body/Body.h>
#include <dynamics/event_driven/ContactSystem.h>
#include <dynamics/time_stepping/SoftContact.h>
#include <dynamics/time_stepping/SolverBody.h>
#include <joints/Bond.h>
#include <math/ProfileMatrix.h>
#include <math/StepMemory.h>

#include <cstdint>
#include <cstdio>
#include <vector>

using namespace kp;

template<class T> void layout(const char* name) {
    std::printf("type,%s,%zu,%zu\n", name, sizeof(T), alignof(T));
}

// The arena promises 16 bytes at the start of an allocation. Individual elements
// only require alignof(T). Check a spill to the next 4 MiB block and a rewind.
bool arrays() {
    StepArray<Vector3> empty;
    StepArray<Vector3> small(33);
    const auto address = [](const auto* p) { return reinterpret_cast<std::uintptr_t>(p); };
    if (!empty.empty() || address(small.data()) % 16) return false;
    size_t aligned = 0;
    for (auto& v : small) {
        if (address(&v) % alignof(Vector3)) return false;
        aligned += address(&v) % 16 == 0;
    }
    small[0].x = 17;
    {
        StepMemory::Scope scope;
        StepArray<Vector3> large((size_t(1) << 22) / sizeof(Vector3) + 1);
        if (address(large.data()) % 16) return false;
        large.back().z = 29;
        if (small[0].x != 17 || large.back().z != 29) return false;
    }
    StepArray<Vector3> after(1);
    after[0].x = 42;
    std::printf("array,addresses_divisible_by_16,%zu,%zu\n", aligned, small.size());
    return address(after.data()) % 16 == 0 && small[0].x == 17 && after[0].x == 42;
}

int main() {
    std::puts("kind,name,value,alignment_or_count");
    layout<Real>("Real");
    layout<Vector3>("Vector3");
    layout<Quaternion>("Quaternion");
    layout<Matrix3x3>("Matrix3x3");
    layout<SolverBody>("SolverBody");
    layout<Body>("Body");
    layout<Bond>("Bond");
    layout<SoftContact>("SoftContact");
    layout<ProfileMatrix>("ProfileMatrix");
    layout<ContactSystem::Line>("ContactSystem::Line");
    layout<ContactSystem::Entry>("ContactSystem::Entry");
    if (!arrays()) return 1;
    {
        StepMemory step;
        if (!arrays()) return 1;
    }
    return StepMemory::held() == 0 ? 0 : 1;
}
