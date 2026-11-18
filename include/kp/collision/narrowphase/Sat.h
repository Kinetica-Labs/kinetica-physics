#pragma once
// Теорема о разделяющей оси для двух выпуклых многогранников.
//
// Два выпуклых многогранника не пересекаются тогда и только тогда, когда их проекции на
// какую-то ось не перекрываются, и такую ось достаточно искать среди нормалей граней обоих
// тел и векторных произведений пар их рёбер. Перебор конечен, поэтому ответ точен: если
// разделяющей оси нет, находится ось наименьшего перекрытия — по ней тела нужно раздвинуть
// меньше всего.
//
// Пары рёбер отсеиваются по карте Гаусса: ось из пары рёбер имеет смысл, только если эти
// рёбра образуют грань разности Минковского. Gregorius, «The Separating Axis Test between
// ConvexShape Polyhedra», Game Developers Conference, 2013.

#include <kp/collision/narrowphase/TransformedHull.h>
#include <kp/math/Transform.h>

namespace kp {

// Ось грани тела B или пары рёбер принимается, только если она лучше оси грани тела A больше чем
// на этот запас. У параллельных граней оси совпадают с точностью до округления, и без запаса
// выбор прыгал бы между ними от шага к шагу — а с ним и номера точек контакта.
constexpr Real kFacePreference = Real(1e-9); // м

struct SeparatingAxis {
    enum class Kind { FaceA, FaceB, EdgePair };
    Kind kind = Kind::FaceA;
    // Положительное — зазор вдоль оси (тела разделены, но это не расстояние между ними:
    // расстояние даёт Gjk.h). Отрицательное — глубина перекрытия.
    Real separation = 0;
    Vector3 normal;      // единичная, от A к B, мир
    int featureA = 0; // грань A; для пары рёбер — ребро A
    int featureB = 0; // грань B; для пары рёбер — ребро B
};

SeparatingAxis findSeparatingAxis(const ConvexHull& a, const Transform& poseA, const ConvexHull& b, const Transform& poseB, Real preference = kFacePreference);

// Лучшая ось только среди нормалей граней. Нужна контакту почти параллельных граней: точная ось
// там — пара рёбер, но касаются тела всей площадкой (HullVsHullAlgorithm).
SeparatingAxis findFaceAxis(const ConvexHull& a, const Transform& poseA, const ConvexHull& b, const Transform& poseB, Real preference = kFacePreference);

// Ось наружной грани (ConvexHull::outerFace), если она есть у одного из тел: зазор — от плоскости
// этой грани до самой глубокой вершины другого тела. Тела даны в общих осях; false — наружной
// грани нет ни у одного.
bool outerFaceAxis(const TransformedHull& A, const TransformedHull& B, SeparatingAxis& axis);

} // namespace kp
