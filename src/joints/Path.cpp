#include <joints/Path.h>

#include <math/math_functions.h>

#include <algorithm>

namespace kp {
namespace {

constexpr int kSteps = 16;             // узлов таблицы на отрезок
constexpr int kProbes = 8;             // проб на отрезок в грубом поиске ближайшей точки
constexpr Real kSettled = Real(1e-14); // шаг метода Ньютона по u, после которого ответ не уточняется

// Зеркальное отражение v в плоскости с нормалью across; нулевая нормаль — без отражения.
Vector3 reflected(Vector3 v, Vector3 across) {
    const Real square = across.dot(across);
    return square > 0 ? v - (2 * across.dot(v) / square) * across : v;
}

// Вектор v поперёк единичной оси, повёрнутый вокруг неё на угол angle.
Vector3 turned(Vector3 v, Vector3 axis, Real angle) {
    const SinCos turn = sinCos(angle);
    return turn.cos * v + turn.sin * axis.cross(v);
}

// Единичный вектор поперёк единичного t: от той оси, вдоль которой t идёт меньше всего.
Vector3 anyAcross(Vector3 t) {
    const Vector3 axis = abs(t.x) <= abs(t.y) && abs(t.x) <= abs(t.z) ? Vector3{1, 0, 0} : abs(t.y) <= abs(t.z) ? Vector3{0, 1, 0} : Vector3{0, 0, 1};
    const Vector3 across = axis - axis.dot(t) * t;
    return (1 / across.length()) * across;
}

} // namespace

Path::Path(std::vector<Vector3> through, bool closed) : through_(std::move(through)), closed_(closed), segments_(int(through_.size()) - (closed ? 0 : 1)) {
    tabulateLength();
    boundSegments();
    transportNormal();
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

// Длина дуги от from до to — интеграл |P′| по u — по трём узлам Гаусса (точно для многочленов до пятой
// степени; правило Симпсона с теми же тремя вычислениями ошибалось на извилистой кривой до 6e-6 м).
// from и to не дальше шага таблицы.
Real Path::stretch(Real from, Real to) const {
    const Real middle = (from + to) / 2, half = (to - from) / 2, offset = half * Real(0.77459666924148338); // √(3/5)
    return (half / 9) * (5 * at(middle - offset).first.length() + 8 * at(middle).first.length() + 5 * at(middle + offset).first.length());
}

void Path::tabulateLength() {
    arc_.assign(size_t(segments_ * kSteps) + 1, 0);
    for (size_t k = 0; k + 1 < arc_.size(); ++k) arc_[k + 1] = arc_[k] + stretch(Real(k) / kSteps, Real(k + 1) / kSteps);
}

// До узла — по таблице, от узла до u — тем же правилом.
Real Path::length(Real u) const {
    u = within(u);
    const size_t k = std::min(size_t(u * kSteps), arc_.size() - 2);
    return arc_[k] + stretch(Real(k) / kSteps, u);
}

// Отрезок сплайна лежит в выпуклой оболочке своих точек Безье P₁, P₁ + (P₂ − P₀)/6, P₂ − (P₃ − P₁)/6, P₂.
// Шар с центром в середине хорды P₁P₂, достающий до самой далёкой из них, охватывает отрезок.
void Path::boundSegments() {
    for (int s = 0; s < segments_; ++s) {
        const Vector3 p1 = control(s), p2 = control(s + 1), center = Real(0.5) * (p1 + p2);
        const Vector3 inner[2] = {p1 + (1 / Real(6)) * (p2 - control(s - 1)), p2 - (1 / Real(6)) * (control(s + 2) - p1)};
        balls_.push_back({center, std::max((p1 - center).length(), std::max((inner[0] - center).length(), (inner[1] - center).length()))});
    }
}

// Нормаль переносится вдоль кривой двойным отражением (Wang, Jüttler, Zheng, Liu, «Computation of
// rotation minimizing frames», ACM Trans. Graph. 27, 2008): первое отражение — в плоскости посередине между
// точками from и to, второе совмещает отражённую касательную с касательной в to. Вместе это поворот,
// который переводит касательную в касательную и вокруг неё не крутит.
Vector3 Path::carry(Vector3 normal, Real from, Real to) const {
    const Vector3 chord = at(to).point - at(from).point;
    return reflected(reflected(normal, chord), tangent(to) - reflected(tangent(from), chord));
}

// У замкнутой кривой после круга нормаль повёрнута вокруг касательной на угол −turn_; узел на длине дуги
// s доворачивается на turn_ · s / L, и нормаль смыкается.
void Path::transportNormal() {
    normal_.assign(1, anyAcross(tangent(0)));
    for (size_t k = 0; k + 1 < arc_.size(); ++k) normal_.push_back(carry(normal_[k], Real(k) / kSteps, Real(k + 1) / kSteps));
    if (!closed_) return;
    const Vector3 first = normal_.front(), last = normal_.back();
    turn_ = atan2(last.cross(first).dot(tangent(0)), last.dot(first));
    for (size_t k = 0; k < normal_.size(); ++k) normal_[k] = turned(normal_[k], tangent(Real(k) / kSteps), turn_ * arc_[k] / arc_.back());
}

// От ближайшего узла таблицы — тем же переносом, поэтому между узлами нормаль не интерполируется.
Vector3 Path::normal(Real u) const {
    u = within(u);
    const size_t k = std::min(size_t(u * kSteps), normal_.size() - 2);
    const Vector3 carried = carry(normal_[k], Real(k) / kSteps, u);
    if (turn_ == 0) return carried;
    const Real share = (u * kSteps - Real(k)) * (arc_[k + 1] - arc_[k]) / arc_.back(); // доля длины кривой от узла до u
    return turned(carried, tangent(u), turn_ * share);
}

// Расстояние до ближайшей из заданных точек — граница сверху. Пробы идут только в отрезки, чей шар
// ближе этой границы: цена поиска — одно расстояние на отрезок и пробы в двух-трёх соседних.
Real Path::closest(Vector3 x) const {
    Real best = 0, least = (through_[0] - x).dot(through_[0] - x);
    for (size_t k = 1; k < through_.size(); ++k) {
        const Real gap = (through_[k] - x).dot(through_[k] - x);
        if (gap < least) least = gap, best = Real(k);
    }
    Real reach = sqrt(least);
    for (int s = 0; s < segments_; ++s) {
        const Real far = balls_[size_t(s)].radius + reach;
        if ((balls_[size_t(s)].center - x).dot(balls_[size_t(s)].center - x) >= far * far) continue;
        for (int probe = 1; probe < kProbes; ++probe) {
            const Real u = Real(s) + Real(probe) / kProbes;
            const Vector3 gap = at(u).point - x;
            if (gap.dot(gap) < least) least = gap.dot(gap), best = u, reach = sqrt(least);
        }
    }
    return refine(best, x);
}

// Метод Ньютона по условию (P − x) · P′ = 0 от грубого приближения; шаг не длиннее четверти отрезка.
// На конце открытой кривой шаг наружу оставляет u на месте — это тоже ответ.
Real Path::refine(Real u, Vector3 x) const {
    for (int iteration = 0; iteration < 12; ++iteration) {
        const Jet jet = at(u);
        const Real slope = jet.first.dot(jet.first) + (jet.point - x).dot(jet.second);
        if (!(slope > 0)) break; // вогнутость: шаг Ньютона увёл бы от ближайшей точки
        const Real step = std::min(std::max((jet.point - x).dot(jet.first) / slope, Real(-0.25)), Real(0.25));
        const Real next = within(u - step);
        const bool settled = next == u || abs(step) <= kSettled;
        u = next;
        if (settled) break;
    }
    return u;
}

} // namespace kp
