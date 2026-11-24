// Публичный дискретный запрос в метрах. CSV содержит по строке на каждый вызов collide.
// При общей касательной плоскости каждая фигура имеет осевую точку опоры и внутренний
// касательный шар радиусом ≥ 0,04 м. Плоскость и два таких шара дают одинаковые
// границы расстояния / малой глубины при осевом сдвиге. Оракул равен сдвигу gap.
#include <collision/ContactPair.h>
#include <collision/narrowphase/NarrowPhaseAlgorithm.h>
#include <collision/shapes/Capsule.h>
#include <collision/shapes/ConvexHull.h>
#include <collision/shapes/Cylinder.h>
#include <collision/shapes/Sphere.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

namespace {
using namespace kp;
constexpr double lengthTolerance = 2e-7;  // 0,2 мкм на фигурах около 0,5 м
constexpr double normalTolerance = 2e-4;  // норма разности единичных векторов

struct Fixture {
    const char* name;
    const ConvexShape* shape;
    Real height; // расстояние от центра до касательной плоскости в направлении ±Y, м
    Quaternion orientation{};
};

struct Frame { const char* name; Transform pose; };

struct Sample {
    int count;
    double deepest = std::numeric_limits<double>::quiet_NaN();
    Vector3 normal;
    bool valid = true;
};

bool finite(Vector3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

Sample query(const Fixture& a, const Transform& pa, const Fixture& b, const Transform& pb, Real margin) {
    const auto closest = closestPoints(*a.shape, pa, *b.shape, pb);
    const auto& method = NarrowPhaseAlgorithm::between(*a.shape, *b.shape);
    const auto patch = method.collide(*a.shape, pa, *b.shape, pb, closest, margin, kContactDistance, kOnBoundary);
    Sample s{patch.count, std::numeric_limits<double>::quiet_NaN(), patch.normal};
    s.valid = patch.count >= 0 && size_t(patch.count) == patch.points.size();
    if (s.count == 0 || !s.valid) return s;
    s.valid = finite(s.normal) && std::abs(s.normal.length() - 1) <= normalTolerance;
    s.deepest = std::numeric_limits<double>::infinity();
    for (const auto& pt : patch.points) {
        const double d = pt.separation;
        s.valid &= finite(pt.onA) && finite(pt.onB) && std::isfinite(d);
        s.valid &= std::abs((pt.onB - pt.onA).dot(s.normal) - d) <= lengthTolerance;
        s.valid &= d <= double(margin) + lengthTolerance;
        s.deepest = std::min(s.deepest, d);
    }
    return s;
}

Transform placed(const Frame& frame, const Fixture& shape, Real y) {
    return {frame.pose.point({0, y, 0}), (frame.pose.orientation * shape.orientation).unit()};
}

bool equivalent(const Sample& a, const Sample& b, bool boundary) {
    if (!a.count || !b.count) return boundary || a.count == b.count;
    return std::abs(a.deepest - b.deepest) <= 2 * lengthTolerance;
}

int study(const Fixture& a, const Fixture& b, Real margin, const std::vector<Real>& gaps,
          const std::vector<Frame>& frames, bool concentric = false) {
    const std::size_t n = gaps.size();
    std::vector<Sample> baseline(n);
    int failures = 0;
    for (std::size_t k = 0; k < frames.size(); ++k) {
        const Frame& frame = frames[k];
        std::vector<Sample> forward(n);
        for (int swap = 0; swap <= 1; ++swap) {
            std::vector<Sample> path(n);
            for (std::size_t i = 0; i < n; ++i) {
                const Real gap = gaps[i];
                const Transform pa = placed(frame, a, 0);
                const Transform pb = placed(frame, b, concentric ? 0 : a.height + b.height + gap);
                const Sample s = swap ? query(b, pb, a, pa, margin) : query(a, pa, b, pb, margin);
                path[i] = s;
                // На самой границе запаса округление допускает оба ответа о наличии площадки.
                const double roundoff = 64 * std::numeric_limits<Real>::epsilon() *
                                        (1 + frame.pose.position.length() + a.height + b.height);
                const bool boundary = std::abs(gap - margin) <= roundoff;
                const bool hit = s.count > 0;
                const bool classified = boundary || hit == (gap < margin);
                const bool signedGap = std::abs(gap) <= roundoff || (gap > 0 ? s.deepest > 0 : s.deepest < 0);
                const bool depth = !hit || (signedGap && std::abs(s.deepest - gap) <= lengthTolerance);
                const Vector3 axis = frame.pose.direction({0, Real(swap ? -1 : 1), 0});
                const bool direction = !hit || concentric || (s.normal - axis).length() <= normalTolerance;
                if (!swap) forward[i] = s;
                if (k == 0 && !swap) baseline[i] = s;
                const bool reversed = !swap || equivalent(s, forward[i], boundary);
                const bool transformed = equivalent(s, baseline[i], boundary);
                // Один и тот же вход на обратном пути: контакт должен воспроизводиться даже на границе.
                const bool retraced = i <= n / 2 || (s.count == path[n - 1 - i].count && equivalent(s, path[n - 1 - i], false));
                const bool passed = s.valid && classified && depth && direction && reversed && transformed && retraced;
                failures += !passed;
                std::printf("%s,%s,%s,%s,%d,%zu,%.9g,%.9g,%d,%.12g,%d,%d,%d,%d,%d,%d,%d,%d\n",
                            concentric ? "coincident" : "support", a.name, b.name, frame.name, swap, i,
                            double(margin), double(gap), s.count, s.deepest, int(s.valid), int(classified),
                            int(depth), int(direction), int(reversed), int(transformed), int(retraced), int(passed));
            }
        }
    }
    return failures;
}
}

int main() {
    using namespace kp;
    const Sphere sphere(.25);
    const Capsule capsule(.125, .125), collapsedCapsule(0, .125);
    const Cylinder cylinder(.25, .18), rounded(.25, .18, .035);
    const auto box = ConvexHull::box({.18, .25, .14});
    const auto cube = ConvexHull::box({.25, .25, .25});
    const auto prism = ConvexHull::fromPoints({{-.18,-.25,-.14},{.18,-.25,-.14},{0,-.25,.19},
                                               {-.18,.25,-.14},{.18,.25,-.14},{0,.25,.19}});
    // Поворот на 90°: локальная ось Y переходит в горизонтальную.
    const Quaternion sideways = Quaternion{1, 0, 0, 1}.unit();
    const std::array<Fixture, 10> shapes{{
        {"sphere", &sphere, .25}, {"capsule_end", &capsule, .25}, {"capsule_side", &capsule, .125, sideways},
        {"cylinder_cap", &cylinder, .25}, {"cylinder_side", &cylinder, .18, sideways},
        {"rounded_cap", &rounded, .25}, {"rounded_side", &rounded, .18, sideways},
        {"box_face", &box, .25}, {"prism_face", &prism, .25}, {"zero_segment_capsule", &collapsedCapsule, .125}}};
    const Quaternion q = Quaternion{.82,.19,-.31,.42}.unit();
    const Vector3 shift{7.25,-3.75,12.5};
    const std::vector<Frame> frames{{"identity", {}}, {"translated", {shift, {}}},
                                    {"rotated", {{}, q}}, {"rotated_translated", {shift, q}}};
    // Подход и возврат. Проверяем запас 0 и 0,1 мм, обе стороны обоих порогов.
    const std::vector<Real> approach{.002, .0001001, .0001, .0000999, .0000001, 0, -.0000001, -.00001, -.001};
    std::vector<Real> gaps = approach;
    gaps.insert(gaps.end(), approach.rbegin() + 1, approach.rend());
    std::puts("case,shape_a,shape_b,frame,swap,step,margin_m,oracle_gap_m,points,min_separation_m,"
              "witness_ok,presence_ok,depth_ok,normal_ok,swap_ok,rigid_ok,return_ok,pass");
    int failures = 0;
    for (std::size_t a = 0; a < shapes.size(); ++a)
        for (std::size_t b = a; b < shapes.size(); ++b)
            for (Real margin : {Real(0), kContactDistance})
                failures += study(shapes[a], shapes[b], margin, gaps, frames);
    // Центры равных шаров и кубов совпадают. Глубина 0,5 м, направление нормали неоднозначно.
    // Ожидается физический контакт; пустая площадка локализует неподдержанную конфигурацию raw API.
    const Fixture tiedCube{"cube", &cube, .25};
    failures += study(shapes[0], shapes[0], 0, {-.5}, frames, true);
    failures += study(shapes[0], shapes[9], 0, {-.375}, frames, true);
    failures += study(shapes[9], shapes[9], 0, {-.25}, frames, true);
    failures += study(tiedCube, tiedCube, 0, {-.5}, frames, true);
    std::fprintf(stderr, "failed_rows=%d,real_bytes=%zu\n", failures, sizeof(Real));
    return failures ? 1 : 0;
}
