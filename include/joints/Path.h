#pragma once
// Кривая, вдоль которой сочленение по траектории (PathJoint) ведёт тело: гладкий сплайн через заданные
// точки (Catmull, Rom, «A class of local interpolating splines», 1974). На отрезке между точками P₁ и P₂ он
// кубический и зависит ещё от соседних P₀ и P₃:
//   P(t) = ½ (2 P₁ + (P₂ − P₀) t + (2 P₀ − 5 P₁ + 4 P₂ − P₃) t² + (−P₀ + 3 P₁ − 3 P₂ + P₃) t³),
// касательная в заданной точке равна полуразности её соседей. У открытой кривой за концами стоят точки,
// отражённые от концов: точки на одной прямой через равные промежутки дают прямую.
//
// Параметр u — номер отрезка плюс доля пути по нему, от нуля до span(); длина дуги до u — length(u),
// метры. Только сложение, умножение, деление и корень.

#include <math/Vector3.h>

#include <vector>

namespace kp {

class Path {
public:
    // through — точки, через которые идёт кривая, не меньше двух и без совпадающих соседних.
    Path(std::vector<Vector3> through, bool closed);
    bool closed() const { return closed_; }
    Real span() const { return Real(segments_); }
    Vector3 point(Real u) const;
    Vector3 tangent(Real u) const;   // единичная
    Vector3 curvature(Real u) const; // d²P/ds²: смотрит к центру кривизны, длина — 1/радиус, 1/м
    Real length(Real u) const;       // длина дуги от начала кривой до u, м
    Real length() const { return arc_.back(); }
    // Параметр ближайшей к x точки кривой. Поиск с нуля: грубо по восьми пробам на отрезок, затем
    // метод Ньютона по условию (P − x) · P′ = 0. Кривая не должна подходить к самой себе ближе, чем тело
    // от неё отходит: в точке самопересечения ближайшая ветвь не определена.
    Real closest(Vector3 x) const;

private:
    struct Jet {
        Vector3 point, first, second; // P, dP/du, d²P/du²
    };
    Jet at(Real u) const;
    Vector3 control(int index) const;
    Real within(Real u) const; // u в пределах кривой: у замкнутой — по кругу, у открытой — до концов

    std::vector<Vector3> through_;
    bool closed_;
    int segments_;
    std::vector<Real> arc_; // длина дуги в узлах таблицы, kSteps узлов на отрезок
};

} // namespace kp
