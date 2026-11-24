#pragma once
// Независимые измерения по мировым позам кубов; таймер движка их не включает.
#include "../common.h"
#include <array>
#include <numeric>

namespace stress {
constexpr double half = 0.125; // м; при плотности 64 кг/м³ куб весит 1 кг

inline void require(bool condition, const char* message) {
    if (!condition) { std::fprintf(stderr, "%s\n", message); std::exit(3); }
}

inline double bottom(const State& s) {
    const V3 y = unrotate(s.turn, {0, 1, 0});
    return s.at.y - half * (std::abs(y.x) + std::abs(y.y) + std::abs(y.z));
}

inline double cubeOverlap(const SceneSpec& scene, const std::vector<State>& st) {
    std::vector<size_t> ids(st.size() - 1);
    std::iota(ids.begin(), ids.end(), size_t(1)); // пол измеряется отдельно
    std::sort(ids.begin(), ids.end(), [&](size_t a, size_t b) { return st[a].at.x < st[b].at.x; });
    const double diameter = 2 * std::sqrt(3.) * half;
    double depth = 0;
    for (size_t i = 0; i < ids.size(); ++i) {
        const size_t a = ids[i];
        for (size_t j = i + 1; j < ids.size(); ++j) {
            const size_t b = ids[j];
            const V3 gap = st[b].at - st[a].at;
            if (gap.x > diameter) break;
            if (std::abs(gap.y) > diameter || std::abs(gap.z) > diameter) continue;
            depth = std::max(depth, penetration(scene.boxes[a], st[a], scene.boxes[b], st[b]));
        }
    }
    return depth;
}

struct Reading {
    double floorDepth = 0, pairDepth = 0, comHeight = 0, apexDrop = 0, energyJ = 0, freeFallError = 0;
    std::array<double, 4> edge{}; // крайние координаты центров вдоль +X, -X, +Z, -Z
    int floorCount = 0, below = 0, asleep = 0;
};

inline Reading observe(const SceneSpec& scene, const std::vector<State>& st, double time, bool freeFall) {
    require(st.size() == scene.boxes.size() && st.size() == 1001, "lost body");
    Reading r;
    for (size_t i = 1; i < st.size(); ++i) {
        const auto& s = st[i];
        const auto q = s.turn;
        const double q2 = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
        require(std::isfinite(dot(s.at,s.at) + dot(s.v,s.v) + dot(s.w,s.w)) && std::abs(q2-1) < 1e-4,
                "non-finite state or non-unit orientation");
        const double gap = bottom(s);
        r.floorDepth = std::max(r.floorDepth, -gap);
        r.floorCount += gap <= .002;
        r.below += s.at.y < 0;
        r.asleep += s.asleep;
        r.comHeight += s.at.y / 1000;
        const std::array<double,4> extent{s.at.x,-s.at.x,s.at.z,-s.at.z};
        require(std::max(std::abs(s.at.x),std::abs(s.at.z)) + std::sqrt(3.)*half < scene.boxes.front().half.x,
                "cube outside the finite floor");
        for (size_t k = 0; k < extent.size(); ++k) r.edge[k] = std::max(r.edge[k], extent[k]);
        if (freeFall) r.freeFallError = std::max(r.freeFallError, std::abs(s.at.y - (scene.boxes[i].at.y - .5*kGravity*time*time)));
    }
    r.apexDrop = scene.boxes.back().at.y - st.back().at.y;
    r.pairDepth = cubeOverlap(scene, st);
    r.energyJ = energy(scene, st);
    return r;
}
} // namespace stress
