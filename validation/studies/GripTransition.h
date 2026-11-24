#pragma once

#include <world/World.h>
#include <cstdio>

namespace grip {
inline bool equal(kp::Vector3 a, kp::Vector3 b) { return a.x == b.x && a.y == b.y && a.z == b.z; }

inline bool sameSolvedState(const kp::World& a, const kp::World& b) {
    for (size_t i = 0; i < a.bodies().size(); ++i) {
        const auto& p = a.bodies()[i];
        const auto& q = b.bodies()[i];
        if (!equal(p.position,q.position) || !equal(p.linearMomentum,q.linearMomentum) || !equal(p.angularMomentum,q.angularMomentum)) return false;
        if (p.orientation.w != q.orientation.w || p.orientation.x != q.orientation.x ||
            p.orientation.y != q.orientation.y || p.orientation.z != q.orientation.z) return false;
    }
    const auto& f = kp::StepObserver::cachedContactForces(a);
    const auto& other = kp::StepObserver::cachedContactForces(b);
    auto next = other.begin();
    for (const auto& [id, load] : f) {
        if (next == other.end() || next->first < id || id < next->first) return false;
        if (next->second.normal != load.normal || next->second.twist != load.twist || !equal(next->second.tangent, load.tangent)) return false;
        ++next;
    }
    return next == other.end();
}

// После обжатия включаем пробу только в одном мире. Изменившаяся нагрузка требует полного решения.
template<class MakeRig> int checkTransitions(MakeRig makeRig) {
    constexpr kp::Real h = kp::Real(1) / 480;
    struct Change { const char* name; kp::Real squeeze, down, friction; };
    for (auto c : {Change{"release",0,10,0.5}, Change{"reverse",10,-10,0.5}, Change{"zero-friction",10,10,0}}) {
        auto a = makeRig(), b = makeRig();
        for (auto* rig : {&a,&b}) {
            rig->world().soft.coupledFriction = true;
            for (int i = 0; i < 240; ++i)
                if (!rig->advance(h, 10, i < 120 ? 0 : -10)) return 1;
            for (size_t i = 1; i < rig->world().bodies().size(); ++i)
                rig->world().body(kp::BodyId(int(i))).friction = c.friction;
        }
        b.world().soft.reuseEquilibrium = true;
        if (!a.advance(h, c.squeeze, -c.down) || !b.advance(h, c.squeeze, -c.down)) return 1;
        const bool same = sameSolvedState(a.world(), b.world());
        std::printf("%s,%d\n", c.name, int(same));
        if (!same) return 1;
    }
    return 0;
}
} // namespace grip
