#pragma once
// Выпуклый многогранник с явной топологией: вершины, грани, рёбра.
// Топология нужна теореме о разделяющей оси (Sat.h): оси берутся из нормалей граней и из
// пар рёбер, а не из одной лишь опорной функции.

#include <collision/shapes/ConvexShape.h>

#include <vector>

namespace kp {

class ConvexHull final : public ConvexShape {
public:
    struct Face {
        Vector3 normal;           // единичная, наружу
        Real offset = 0;     // normal · x = offset на грани
        std::vector<int> loop; // вершины против часовой стрелки, если смотреть снаружи
        bool internal = false; // вся грань закрыта соседней частью того же составного тела
    };
    struct Edge {
        int tail = 0, head = 0; // вершины; направление — как в обходе грани left
        int left = 0, right = 0; // две грани, которым принадлежит ребро
    };

    // Грани задаются обходами вершин; плоскости и рёбра выводятся из них.
    ConvexHull(std::vector<Vector3> vertices, const std::vector<std::vector<int>>& loops);

    // Брусок с полуразмерами h.
    static ConvexHull box(Vector3 h);

    // Выпуклая оболочка облака точек (QuickHull.cpp). Точки внутри оболочки отбрасываются,
    // треугольники в одной плоскости сливаются в многоугольные грани. tolerance — расстояние, на
    // котором точка считается лежащей в плоскости грани, м; ноль — 10⁻⁹ от размера облака. Для
    // сетки с шумом его стоит задать величиной шума, иначе плоская сторона детали распадётся на
    // треугольники. Бросает std::invalid_argument, если точки лежат в одной плоскости.
    static ConvexHull fromPoints(const std::vector<Vector3>& points, Real tolerance = 0);

    Vector3 support(Vector3 direction) const override;
    Real boundingRadius() const override { return extent_; }
    const ConvexHull* hull() const override { return this; }
    std::optional<Flat> flatTowards(const Transform& pose, Vector3 towards) const override;
    Outline outlineOver(const Transform& pose, const Flat& flat) const override;
    // Грань face как плоский элемент.
    Flat faceFlat(const Transform& pose, int face) const;

    // Единственная грань, которой тело обращено наружу, или −1. Так устроена призма рельефа
    // (CompoundBodyBuilder::ground): её поверхность — сам треугольник, а боковые грани лежат внутри рельефа,
    // вплотную к соседним призмам. Контакт с таким телом идёт только через наружную грань: иначе
    // брусок, скользящий по полу из треугольников, упирался бы в бок следующей призмы на шве.
    int outerFace = -1;

    bool isInternalFace(int face) const { return faces_[size_t(face)].internal; }
    void markInternalFace(int face) { faces_[size_t(face)].internal = true; }
    const std::vector<Vector3>& vertices() const { return vertices_; }
    const std::vector<Face>& faces() const { return faces_; }
    const std::vector<Edge>& edges() const { return edges_; }

private:
    Vector3 coreExtents() const override { return coreExtents_; }
    Vector3 coreExtents_;
    std::vector<Vector3> vertices_;
    std::vector<Face> faces_;
    std::vector<Edge> edges_;
    Real extent_ = 0;
};

} // namespace kp
