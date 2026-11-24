// study-grip-limit [exact|realtime|realtime32|shared8|trace|trace-order]: без аргумента — два штатных режима.
#include "GripLimit.h"
#include <math/WorkerPool.h>
#include <world/World.h>
#include <cstdlib>
#include <string_view>

namespace {
class Rig {
public:
    Rig(int substeps, grip::Load load, bool sharedClock = false, bool rightFirst = false) : sharedClock_(sharedClock) {
        world_.realtime = substeps > 0;
        if (world_.realtime) world_.soft.substeps = substeps;
        if (sharedClock_) world_.soft.contactUpdates = 1;
        world_.gravity = {};
        const auto base = world_.addBody(kp::Body{});
        for (bool right : {rightFirst, !rightFirst}) {
            const auto jaw = right ? grip::rightJaw : grip::leftJaw;
            const auto id = addBox(grip::jawPosition(jaw, load.open), load.friction);
            if (right) right_ = id;
            else left_ = id;
        }
        cube_ = addBox(grip::cube, load.friction);
        for (auto jaw : {left_, right_})
            kp::addSlider(world_, base, jaw, world_.body(jaw).position, {1, 0, 0});
    }
    bool step(double dt, double squeeze, double down) {
        // Прибор передаёт секунды и ньютоны в double. Состояние и шаг мира имеют тип kp::Real.
        const int updates = sharedClock_ ? 8 : 1;
        const kp::Real h = kp::Real(dt / updates);
        const kp::Real jawForce = kp::Real(squeeze), cubeForceY = kp::Real(-down);
        for (int i = 0; i < updates; ++i)
            if (!advance(h, jawForce, cubeForceY)) return false;
        return true;
    }
    bool advance(kp::Real h, kp::Real jawForce, kp::Real cubeForceY) {
        world_.body(left_).force.x = jawForce;
        world_.body(right_).force.x = -jawForce;
        world_.body(cube_).force.y = cubeForceY;
        world_.step(h);
        return !world_.report().stalled && world_.report().unfinished == 0;
    }
    kp::Real cubeMomentumY() const { return world_.body(cube_).linearMomentum.y; }
    // Проекция импульса трения каждой губки на ось y куба в мировых осях, Н·с.
    // Для этого опыта один публичный шаг = один подшаг h, без отскока.
    struct JawImpulses { kp::Real left = 0, right = 0; };
    JawImpulses jawImpulses(kp::Real h) const {
        JawImpulses sum;
        for (const auto& [pair, force] : kp::StepObserver::cachedContactForces(world_)) {
            if (pair.a != int(cube_) && pair.b != int(cube_)) continue;
            const bool onB = pair.b == int(cube_);
            const int jaw = onB ? pair.a : pair.b;
            const kp::Real impulse = (onB ? h : -h) * force.tangent.y;
            if (jaw == int(left_)) sum.left += impulse;
            if (jaw == int(right_)) sum.right += impulse;
        }
        return sum;
    }
    grip::Reading read() const {
        const auto &left = world_.body(left_), &right = world_.body(right_);
        return {world_.body(cube_).position.y - (left.position.y + right.position.y) / 2,
                left.linearMomentum.x, right.linearMomentum.x};
    }
private:
    kp::BodyId addBox(grip::Box b, double friction) {
        const kp::Vector3 half{kp::Real(b.halfX), kp::Real(b.halfY), kp::Real(b.halfZ)};
        auto box = kp::Body::box(half, kp::Real(b.density), {kp::Real(b.x), 0, 0});
        box.friction = kp::Real(friction);
        return world_.addBody(box);
    }
    kp::World world_;
    bool sharedClock_;
    kp::BodyId left_, right_, cube_;
};

// Вверх положительно. Из Δp_y = J_конт − F h восстанавливаем импульс обеих губок.
int traceThreshold(grip::Load load, bool detailed = false, bool rightFirst = false) {
    constexpr kp::Real h = kp::Real(1) / 480;
    Rig rig(1, load, true, rightFirst);
    for (int i = 0; i < 120; ++i)
        if (!rig.advance(h, kp::Real(load.squeeze), 0)) return 1;
    const double start = rig.read().relativeY;
    if (!detailed) std::puts("microstep,time_s,relative_drop_mm,cube_vy_m_s,up_contact_Ns,required_Ns");
    bool closed = true;
    for (int i = 1; i <= 120; ++i) {
        const kp::Real before = rig.cubeMomentumY();
        if (!rig.advance(h, kp::Real(load.squeeze), kp::Real(-load.down))) return 1;
        const double after = rig.cubeMomentumY(), required = load.down * h;
        if (detailed) {
            const auto jaws = rig.jawImpulses(h);
            const double residual = (after - before + required) - (jaws.left + jaws.right);
            std::printf("%s,%.2f,%.1f,%.12g,%.12g,%.12g,", rightFirst ? "right-first" : "left-first",
                        load.friction, load.down, double(jaws.left), double(jaws.right), residual);
            closed &= std::abs(residual) <= 1e-7;
        }
        std::printf("%d,%.12g,%.12g,%.12g,%.12g,%.12g\n", i, double(i * h),
                    1000 * (start - rig.read().relativeY), after / grip::cubeMass, after - before + required, required);
    }
    return closed ? 0 : 1;
}

int traceOrder() {
    std::puts("order,mu,load_N,left_up_Ns,right_up_Ns,balance_error_Ns,"
              "microstep,time_s,relative_drop_mm,cube_vy_m_s,up_contact_Ns,required_Ns");
    int status = 0;
    for (auto load : {grip::Load{0.5,10,10}, grip::Load{0.5,10,-10}, grip::Load{0,10,10}})
        for (bool reversed : {false, true})
            status |= traceThreshold(load, true, reversed);
    return status;
}
}

int main(int argc, char** argv) {
    const std::string_view mode = argc == 2 ? argv[1] : "";
    if (argc > 2 || (argc == 2 && mode != "exact" && mode != "realtime" && mode != "realtime32" && mode != "shared8" && mode != "trace" && mode != "trace-order")) {
        std::fputs("study-grip-limit [exact|realtime|realtime32|shared8|trace|trace-order]\n", stderr);
        return 2;
    }
    if (const char* threads = std::getenv("KP_THREADS")) kp::WorkerPool::shared().resize(std::atoi(threads));
    if (mode == "trace") return traceThreshold({0.5, 10, 10});
    if (mode == "trace-order") return traceOrder();
    grip::header();
    bool pass = true;
    if (argc == 1 || mode == "exact")
        pass &= grip::sweep("kinetica-exact", [](grip::Load load) { return Rig(0, load); });
    if (argc == 1 || mode == "realtime")
        pass &= grip::sweep("kinetica-realtime", [](grip::Load load) { return Rig(kp::SoftStep{}.substeps, load); });
    if (mode == "realtime32")
        pass &= grip::sweep("kinetica-realtime32", [](grip::Load load) { return Rig(32, load); });
    if (mode == "shared8")
        pass &= grip::sweep("kinetica-shared8", [](grip::Load load) { return Rig(1, load, true); });
    return pass ? 0 : 1;
}
