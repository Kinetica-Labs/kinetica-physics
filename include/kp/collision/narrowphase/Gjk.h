#pragma once
// Расстояние между двумя выпуклыми формами: алгоритм Гилберта — Джонсона — Кёрти.
//
// Разность Минковского A − B = {a − b} выпукла, а расстояние между A и B равно расстоянию от
// начала координат до неё. Алгоритм строит внутри разности симплекс (точку, отрезок,
// треугольник, тетраэдр) из опорных точек и каждый раз сдвигает его к началу координат.
// Симплекс живёт в пространстве разности: это не тетраэдр поверхности тела. Из него берутся
// веса, по которым восстанавливаются ближайшие точки на самих телах.
//
// Gilbert, Johnson, Keerthi, «A fast procedure for computing the distance between complex
// objects in three-dimensional space», IEEE J. Robotics and Automation 4(2), 1988.

#include <kp/collision/shapes/ConvexShape.h>
#include <kp/math/Transform.h>

namespace kp {

struct ClosestPoints {
    // Расстояние между поверхностями форм, с учётом радиусов. Отрицательное — скруглённые
    // оболочки перекрылись, но ядра ещё разделены: глубина известна точно.
    Real distance = 0;
    Vector3 onA, onB; // ближайшие точки поверхностей, мир
    Vector3 normal;   // единичная, от A к B
    // Ядра касаются или перекрылись: расстояние и нормаль не определены, для многогранников
    // дальше работает теорема о разделяющей оси (Sat.h). У пары с цилиндром так бывает только при
    // перекрытии глубже самого цилиндра: его ядро ужимается, пока глубина не найдётся (Gjk.cpp).
    bool coresOverlap = false;
    int iterations = 0;
};

ClosestPoints closestPoints(const ConvexShape& a, const Transform& poseA, const ConvexShape& b, const Transform& poseB);

} // namespace kp
