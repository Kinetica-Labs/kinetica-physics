#pragma once
// Кривая, вдоль которой сочленение по траектории (PathJoint) ведёт тело: кубический сплайн через заданные
// точки с непрерывными касательной и кривизной (de Boor, «A practical guide to splines», 1978). На отрезке
// между точками P₀ и P₁ с производными m₀ и m₁ в них
//   P(t) = P₀ + m₀ t + (3 (P₁ − P₀) − 2 m₀ − m₁) t² + (2 (P₀ − P₁) + m₀ + m₁) t³,
// а производные находятся из условия, что вторая производная в точке одна с обеих сторон:
//   m_{k−1} + 4 m_k + m_{k+1} = 3 (P_{k+1} − P_{k−1});
// у открытой кривой на концах она нулевая: 2 m₀ + m₁ = 3 (P₁ − P₀). Точки на одной прямой через равные
// промежутки дают прямую. Кривизна важна физике: от неё центростремительная сила тела на кривой. Сплайн
// Катмулла — Рома, стоявший здесь сначала, её рвал: на окружности через 8 точек она гуляла от 0,81 до 1,5.
//
// Параметр u — номер отрезка плюс доля пути по нему, от нуля до span(); длина дуги до u — length(u),
// метры. Только сложение, умножение, деление и корень; синус и арктангенс — свои (math/).

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
    // Единичный вектор поперёк касательной, идущий вдоль кривой без кручения: он поворачивается только
    // вместе с касательной, вокруг неё — нет. Особых направлений у него нет, в отличие от любого правила
    // «нормаль по одной касательной». У замкнутой кривой после круга он не вернулся бы в себя: остаток
    // поворота (не больше π) распределён по длине равномерно.
    Vector3 normal(Real u) const;
    // Угловая скорость тройки (касательная, нормаль, их произведение) на метр пути, рад/м: T × dT/ds и, у
    // замкнутой кривой, распределённый остаток поворота вокруг T.
    Vector3 turning(Real u) const;
    Vector3 turningChange(Real u) const; // её производная по длине дуги, рад/м²
    Real length(Real u) const; // длина дуги от начала кривой до u, м
    Real length() const { return arc_.back(); }
    // Параметр ближайшей к x точки кривой. Поиск с нуля, без памяти о прошлом ответе: ближайшая из заданных
    // точек, затем пробы в отрезках, которые могут быть ближе, затем метод Ньютона по условию
    // (P − x) · P′ = 0. Кривая не должна подходить к самой себе ближе, чем тело от неё отходит: в точке
    // самопересечения ближайшая ветвь не определена.
    Real closest(Vector3 x) const;

private:
    struct Jet {
        Vector3 point, first, second, third; // P, dP/du, d²P/du², d³P/du³
    };
    struct Ball {
        Vector3 center;
        Real radius;
    };
    Jet at(Real u) const;
    size_t knot(int index) const; // номер заданной точки: у замкнутой кривой — по кругу
    Real within(Real u) const; // u в пределах кривой: у замкнутой — по кругу, у открытой — до концов
    Real stretch(Real from, Real to) const;
    Real refine(Real u, Vector3 x) const;
    Vector3 carry(Vector3 normal, Real from, Real to) const;
    void solveSlopes();
    void tabulateLength();
    void boundSegments();
    void transportNormal();

    std::vector<Vector3> through_, slope_; // заданные точки и производные dP/du в них
    bool closed_;
    int segments_;
    std::vector<Real> arc_;       // длина дуги в узлах таблицы, kSteps узлов на отрезок
    std::vector<Vector3> normal_; // нормаль в тех же узлах
    Real turn_ = 0;               // замкнутая кривая: на какой угол довёрнута нормаль за круг, рад
    std::vector<Ball> balls_;     // шар, охватывающий отрезок
};

} // namespace kp
