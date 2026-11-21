#include <joints/Path.h>

#include <math/math_functions.h>

#include <algorithm>

namespace kp {
namespace {

constexpr int kSteps = 16; // узлов таблицы длины дуги на отрезок
constexpr int kProbes = 8; // проб на отрезок в грубом поиске ближайшей точки

} // namespace

// Длина дуги — интеграл |P′| по u; на шаге таблицы он берётся по Симпсону.
Path::Path(std::vector<Vector3> through, bool closed) : through_(std::move(through)), closed_(closed), segments_(int(through_.size()) - (closed ? 0 : 1)) {
    arc_.assign(size_t(segments_ * kSteps) + 1, 0);
    const Real h = Real(1) / kSteps;
    for (size_t k = 0; k + 1 < arc_.size(); ++k) {
        const Real u = Real(k) * h;
        arc_[k + 1] = arc_[k] + (h / 6) * (at(u).first.length() + 4 * at(u + h / 2).first.length() + at(std::min(u + h, span())).first.length());
    }
}

Vector3 Path::control(int index) const {
    const int n = int(through_.size());
    if (closed_) return through_[size_t(((index % n) + n) % n)];
    if (index < 0) return 2 * through_[0] - through_[1];
    if (index >= n) return 2 * through_[size_t(n - 1)] - through_[size_t(n - 2)];
    return through_[size_t(index)];
}

Real Path::within(Real u) const {
    if (!closed_) return std::min(std::max(u, Real(0)), span());
    u -= span() * Real(int(u / span()));
    return u < 0 ? u + span() : u;
}

Path::Jet Path::at(Real u) const {
    u = within(u);
    const int segment = std::min(int(u), segments_ - 1); // на конце открытой кривой — последний отрезок, t = 1
    const Real t = u - Real(segment);
    const Vector3 p0 = control(segment - 1), p1 = control(segment), p2 = control(segment + 1), p3 = control(segment + 2);
    const Vector3 b = p2 - p0, c = 2 * p0 - 5 * p1 + 4 * p2 - p3, d = 3 * p1 - p0 - 3 * p2 + p3;
    return {Real(0.5) * (2 * p1 + t * b + (t * t) * c + (t * t * t) * d), Real(0.5) * (b + (2 * t) * c + (3 * t * t) * d), c + (3 * t) * d};
}

Vector3 Path::point(Real u) const { return at(u).point; }

Vector3 Path::tangent(Real u) const {
    const Vector3 first = at(u).first;
    return (1 / first.length()) * first;
}

// d²P/ds² = (P″ − (P″ · T) T) / |P′|²: вторая производная по длине дуги — часть P″ поперёк касательной.
Vector3 Path::curvature(Real u) const {
    const Jet jet = at(u);
    const Real speed = jet.first.length();
    const Vector3 t = (1 / speed) * jet.first;
    return (1 / (speed * speed)) * (jet.second - jet.second.dot(t) * t);
}

Real Path::length(Real u) const {
    const Real place = within(u) * kSteps;
    const size_t k = std::min(size_t(place), arc_.size() - 2);
    return arc_[k] + (place - Real(k)) * (arc_[k + 1] - arc_[k]);
}

Real Path::closest(Vector3 x) const {
    Real best = 0, least = (at(0).point - x).dot(at(0).point - x);
    for (int k = 1; k <= segments_ * kProbes - (closed_ ? 1 : 0); ++k) {
        const Real u = Real(k) / kProbes;
        const Vector3 gap = at(u).point - x;
        if (gap.dot(gap) < least) least = gap.dot(gap), best = u;
    }
    for (int iteration = 0; iteration < 12; ++iteration) {
        const Jet jet = at(best);
        const Real slope = jet.first.dot(jet.first) + (jet.point - x).dot(jet.second);
        if (!(slope > 0)) break; // вогнутость: шаг Ньютона увёл бы от ближайшей точки
        const Real step = (jet.point - x).dot(jet.first) / slope;
        best = within(best - std::min(std::max(step, Real(-0.25)), Real(0.25)));
    }
    return best;
}

} // namespace kp
