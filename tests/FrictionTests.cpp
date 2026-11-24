#include "Check.h"
#include <dynamics/time_stepping/CoupledFriction.h>
#include <dynamics/time_stepping/FrictionIslands.h>

namespace {
using namespace kp;
constexpr Real h = Real(1) / 480;

// Вход в проход трения: нормали уже решены, их импульсы заданы в Н·с.
struct Fixture {
    explicit Fixture(size_t count) : bodies(count), states(count) {
        for (auto& body : bodies) body.friction = 1;
    }
    void point(int a, int b, Vector3 normal, Vector3 position, Real load, Real gap = 0, Vector3 warm = {}) {
        ContactPair pair;
        pair.a = a;
        pair.b = b;
        pair.manifold.normal = normal;
        pair.manifold.add({position, position + gap * normal, gap, 0});
        ContactForces::Entry previous{{a,b,0,0,0},{load/h,(1/h)*warm,0}};
        contacts.emplace_back(pair, bodies, states, Softness::rigid(), Span<ContactForces::Entry>{&previous,1}, h, false, false);
    }
    std::vector<Body> bodies;
    StepArray<SolverBody> states;
    StepArray<SoftContact> contacts;
};

struct CornerResult { Real vx, spin; size_t bytes; };

CornerResult rotatingCorner(size_t bodies, bool gap) {
    Fixture f(bodies);
    f.states[1].inverseMass = 1;
    f.states[1].inverseInertia = Matrix3x3::diagonal({1,1,1});
    f.states[1].w.z = 1;
    f.bodies[2].friction = 0;
    for (size_t i = 4; i < bodies; ++i) f.states[i].inverseMass = 1;
    f.point(0,1,{0,1,0},{0,-1,0},2);
    f.point(2,1,{1,0,0},{-1,0,0},1);
    if (gap) f.point(3,1,{0,0,1},{0,0,-1},0,Real(5e-5));
    StepMemory memory;
    ContactLevels levels;
    const size_t start = StepMemory::held();
    solveFrictionIslands(f.contacts, f.states, DirectJoints{}, {}, levels, h, 0);
    return {f.states[1].v.x, f.states[1].w.z, StepMemory::held()-start};
}

void frictionlessLightBody() {
    Fixture f(3);
    f.states[1].inverseMass = 1;
    f.states[2].inverseMass = Real(1e6);
    f.bodies[2].friction = 0;
    f.point(0,1,{0,1,0},{},2);
    f.point(1,2,{0,1,0},{},1,0,{Real(0.25),0,0});
    f.states[1].v.x = Real(0.75);  // до тёплого старта: тяжёлое 1 м/с, лёгкое 2 м/с
    f.states[2].v.x = Real(250002);
    CoupledFriction pair(f.contacts[0], f.contacts[1]);
    CHECK(pair.solve(f.states, DirectJoints{}, h, 0), "zero-radius contact pair");
    CHECK_NEAR(f.states[1].v.x, 0, 1e-8);
    CHECK_NEAR(f.states[2].v.x, 2, 1e-8);
    ContactForces memory;
    f.contacts[1].remember(memory, h);
    CHECK_NEAR(h * memory.begin()->second.tangent.length(), 0, 1e-12);
}
}

void frictionTests() {
    frictionlessLightBody();
    const auto bare = rotatingCorner(4, false);
    const auto gap = rotatingCorner(4, true);
    const auto sparse = rotatingCorner(10004, true);
    CHECK(bare.vx >= -1e-9 && bare.spin < 1, "closing speed or zero friction in loaded corner");
    CHECK_NEAR(gap.vx, bare.vx, 1e-9);
    CHECK_NEAR(gap.spin, bare.spin, 1e-9);
    CHECK_NEAR(sparse.spin, gap.spin, 0);
    CHECK(sparse.bytes <= gap.bytes + 256, "disconnected bodies grew the friction arena: %zu -> %zu", gap.bytes, sparse.bytes);
}
