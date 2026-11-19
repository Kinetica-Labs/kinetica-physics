#pragma once
// Выпуклая форма как «ядро плюс радиус»: множество точек не дальше radius от выпуклого ядра.
// Шар — точка с радиусом, капсула — отрезок с радиусом, многогранник — он сам с нулевым.
//
// Ядро задано опорной функцией: support(d) — точка ядра, самая далёкая в направлении d.
// Расстояние между формами равно расстоянию между ядрами минус оба радиуса, поэтому
// скруглённым формам не нужен отдельный алгоритм — хватает расстояния между ядрами.

#include <math/Transform.h>

#include <optional>

namespace kp {

class ConvexHull;
struct Flat;
struct Outline;
struct Tube;

class ConvexShape {
public:
    virtual ~ConvexShape() = default;
    // Направление и результат — в осях формы. Направление не обязано быть единичным.
    virtual Vector3 support(Vector3 direction) const = 0;
    // Наибольшее расстояние от начала осей формы до её поверхности: радиус охватывающего шара.
    virtual Real boundingRadius() const = 0;

    // Чем форма упирается в касательную плоскость (collision/narrowphase): по этим элементам
    // строятся точки контакта. pose — поза формы в мире, направления — в мире.
    // Многогранник это или форма со скруглением.
    virtual const ConvexHull* hull() const { return nullptr; }
    // Плоский элемент, которым форма обращена в сторону towards; у шара, капсулы и цилиндра на
    // боку такого нет.
    virtual std::optional<Flat> flatTowards(const Transform& pose, Vector3 towards) const;
    // Контур, обращённый к плоскому элементу flat другого тела; у шара — его центр.
    virtual Outline outlineOver(const Transform& pose, const Flat& flat) const;
    // Радиус качения формы, обращённой в сторону towards: на сколько под поверхностью лежит
    // точка, которая движется вместе с телом. У шара и капсулы это их радиус; у цилиндра на
    // боку — весь его радиус (точка на оси), а на торце — только скругление рёбер.
    virtual Real rollingRadius(const Transform& pose, Vector3 towards) const;
    // Трубка, которой форма обращена в сторону towards: капсула или цилиндр боком.
    virtual std::optional<Tube> tubeTowards(const Transform& pose, Vector3 towards) const;

    Real radius = 0; // м
};

} // namespace kp
