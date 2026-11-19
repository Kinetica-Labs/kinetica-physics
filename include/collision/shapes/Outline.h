#pragma once
// Контур, которым тело обращено к плоскому элементу другого тела.

#include <math/Vector3.h>

#include <vector>

namespace kp {

// Контур, которым тело обращено к плоскому элементу другого тела.
struct Outline {
    enum class Kind { Point, Segment, Loop, Circle } kind = Kind::Point;
    std::vector<Vector3> points; // Point — одна; Segment — два конца; Loop — вершины грани; Circle — четыре точки окружности
    Vector3 normal, centre;      // Loop и Circle: нормаль плоскости контура наружу тела; Circle: центр
    Real radius = 0;          // Circle
    Real rounding = 0;        // поверхность тела дальше точек ядра на радиус скругления

    bool closed() const { return kind == Kind::Loop || kind == Kind::Circle; }
};

} // namespace kp
