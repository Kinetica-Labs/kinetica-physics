// Две коробки с известной площадью перекрытия проекций. Верхняя движется вниз со скоростью 1 м/с.
// При положительной ширине w первый физический контакт наступает в t = gap / speed.
#include <collision/ContactPair.h>
#include <algorithm>
#include <cstdio>
#include <limits>

namespace {
using kp::Real;
void measure(Real width, Real gap, Real reach, Real boundary) {
    constexpr Real half = Real(0.05);
    const kp::Vector3 size{half, half, half};
    const auto lower = kp::Body::box(size, 0, {0, -half, 0});
    auto upper = kp::Body::box(size, 1000, {2 * half - width, half + gap, 0});
    upper.linearMomentum.y = -1 / upper.mass.inverseMass;
    const auto pair = kp::examinePairWithin(lower, upper, reach, boundary);
    const int count = pair ? pair->manifold.count : 0;
    Real left = std::numeric_limits<Real>::infinity(), right = -left, separation = left;
    for (int i = 0; i < count; ++i) {
        const auto& point = pair->manifold.points[i];
        left = std::min(left, point.onA.x);
        right = std::max(right, point.onA.x);
        separation = std::min(separation, point.separation);
    }
    std::printf("%.5g,%.5g,%.5g,%.5g,%d,%d,", double(1000*width), double(1000*gap),
                double(1000*reach), double(1000*boundary), int(bool(pair)), count);
    if (count) std::printf("%.9g,%.9g", double(1000*(right-left)), double(1000*separation));
    std::putchar('\n');
}
}

int main() {
    std::puts("oracle_width_mm,gap_mm,reach_mm,edge_tolerance_mm,candidate,points,patch_span_x_mm,min_separation_mm");
    for (Real width : {Real(0.05), Real(0.0005), Real(0.00005)})
        for (Real gap : {Real(0), Real(0.00005), Real(0.0005), Real(0.003)})
            for (Real reach : {Real(0), Real(0.001)})
                for (Real boundary : {Real(0), Real(0.001)}) measure(width, gap, reach, boundary);
}
