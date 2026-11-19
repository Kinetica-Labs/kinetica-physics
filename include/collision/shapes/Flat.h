#pragma once
// Плоский элемент, которым тело упирается в касательную плоскость.

#include <math/Vector3.h>

#include <vector>

namespace kp {

// Точка, лежащая на границе отсечения с такой точностью, считается оставшейся внутри. Когда
// грани двух тел совпадают по контуру, вершины попадают на границу с точностью до округления;
// без запаса шум то оставлял бы вершину, то заменял её точкой пересечения с другим номером.
constexpr Real kOnBoundary = Real(1e-9); // м

// Наклон, меньше которого две плоскости считаются параллельными: у параллельных нет «самой низкой»
// точки окружности, и отсчёт её точек идёт от направления, связанного с телом.
constexpr Real kFaceAligned = Real(1e-6);

// Плоский элемент тела: грань многогранника или круг торца цилиндра.
struct Flat {
    Vector3 normal, origin;       // нормаль наружу и точка плоскости ядра
    Real rounding = 0;         // поверхность тела выше плоскости ядра на радиус скругления
    std::vector<Vector3> corners; // контур грани против часовой стрелки, если смотреть снаружи; пусто — круг
    Vector3 centre, side;         // круг: центр и направление в его плоскости, связанное с телом
    Real radius = 0;

    bool disc() const { return corners.empty(); }
    Vector3 flatten(Vector3 v) const { return v - v.dot(normal) * normal; }
    Real height(Vector3 p) const { return normal.dot(p - origin); }
    // Расстояние точки до стороны k наружу от контура грани; отрицательное — внутри.
    Real outside(size_t k, Vector3 p) const {
        const Vector3 tail = corners[k], out = (corners[(k + 1) % corners.size()] - tail).cross(normal);
        return out.dot(p - tail) / out.length();
    }
    // Лежит ли точка над элементом: её проекция вдоль нормали — внутри контура.
    bool contains(Vector3 p) const {
        if (disc()) return flatten(p - centre).length() <= radius + kOnBoundary;
        for (size_t k = 0; k < corners.size(); ++k)
            if (outside(k, p) > kOnBoundary) return false;
        return true;
    }

    // Четыре точки окружности через четверть оборота, первая — в направлении first.
    static std::vector<Vector3> quarterPoints(Vector3 centre, Vector3 normal, Real radius, Vector3 first) {
        const Vector3 second = normal.cross(first);
        return {centre + radius * first, centre + radius * second, centre - radius * first, centre - radius * second};
    }

    // Единичное направление v в плоскости с нормалью normal; fallback — если v к ней перпендикулярно.
    static Vector3 inPlane(Vector3 v, Vector3 normal, Vector3 fallback) {
        const Vector3 flat = v - v.dot(normal) * normal;
        return flat.length() > kFaceAligned ? (1 / flat.length()) * flat : fallback;
    }
};

} // namespace kp
