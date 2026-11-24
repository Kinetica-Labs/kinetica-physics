// Compare the short grasp with longer holds and an unforced connected pair on the same clock.
#define main originalGripMain
#include "GripLimit.cpp"
#undef main
#include <array>
#include <cstdio>

namespace {
using namespace kp;
constexpr Real h = Real(1) / 480;
void grasp(bool loaded) {
    Rig rig(1, {0.5, 10, 10}, true);
    World& world = rig.world();
    world.sleeping = true;
    world.soft.coupledFriction = true;
    std::array<int, 3> quiet{}, atShort{}, first{};
    int shortAsleep = 0, firstAll = 0, changes = 0, previous = 0;
    double maxSpeed = 0;
    for (int k = 1; k <= 1440; ++k) {
        if (!rig.advance(h, 10, loaded && k > 120 ? -10 : 0)) std::exit(2);
        for (int i = 0; i < 3; ++i) {
            const BodyId id{i + 1};
            const Body& body = world.body(id);
            const Real speed = body.linearVelocity().length() + body.angularVelocity().length() * body.boundingRadius();
            if (!world.asleep(id)) {
                quiet[i] = speed < world.sleepSpeed ? quiet[i] + 1 : 0;
            } else if (!first[i]) first[i] = k;
            if (k == 240) atShort[i] = quiet[i];
            if (k <= 240) maxSpeed = std::max(maxSpeed, double(speed));
        }
        const int now = world.report().asleep;
        if (k <= 240) shortAsleep += now > 0;
        if (now == 3 && !firstAll) firstAll = k;
        if (now != previous) ++changes;
        previous = now;
    }
    std::printf("%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.12g\n", loaded ? "loaded" : "squeeze",
                shortAsleep, atShort[0], atShort[1], atShort[2], first[0], first[1], first[2],
                firstAll, changes, previous, maxSpeed);
}
}
int main() {
    kp::WorkerPool::shared().resize(1);
    std::puts("case,short_steps_with_sleep,quiet_steps_left_at_240,quiet_steps_right_at_240,quiet_steps_cube_at_240,first_sleep_left,first_sleep_right,first_sleep_cube,first_all,changes_asleep_count,asleep_at_1440,peak_short_speed_m_s");
    grasp(true);
    grasp(false);
    World world;
    world.gravity = {};
    world.sleeping = true;
    const auto a = world.addBody(Body::sphere(.05, 1000, {0,0,0}));
    const auto b = world.addBody(Body::sphere(.05, 1000, {.3,0,0}));
    addWeld(world, a, b, {.15,0,0});
    int firstBoth = 0;
    for (int i = 1; i <= 480; ++i) {
        world.step(h);
        if (world.asleep(a) && world.asleep(b) && !firstBoth) firstBoth = i;
    }
    world.body(a).force = {1,0,0};
    world.step(h);
    std::fprintf(stderr,"passive_weld first_both_sleep_step=%d sleep_s=%.12g asleep_after_force=%d velocity_a=%.12g velocity_b=%.12g\n",
        firstBoth, double(firstBoth*h), world.report().asleep, double(world.body(a).linearVelocity().x), double(world.body(b).linearVelocity().x));
    return firstBoth >= 240 && firstBoth <= 241 && world.report().asleep == 0 &&
        world.body(a).linearVelocity().x > 0 && world.body(b).linearVelocity().x > 0 ? 0 : 3;
}
