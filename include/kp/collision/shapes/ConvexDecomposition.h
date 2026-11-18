#pragma once
// Разбиение невыпуклого тела на выпуклые части.
//
// Движок сталкивает только выпуклые формы, поэтому невыпуклое тело — тор, ведро, чайник —
// представляется суммой выпуклых частей (CompoundBodyBuilder::compound). Точное разбиение многогранника по всем
// вогнутым рёбрам даёт сотни осколков; здесь разбиение приближённое, с заданным допуском, в духе
// V-HACD и CoACD:
//
//   1. тело заполняется кубиками-вокселями (внутри или снаружи — по чётности пересечений луча);
//   2. область вокселей, дальше всех отстоящая от выпуклой, режется плоскостью вдоль осей — и так
//      рекурсивно, пока каждая область не станет почти выпуклой: её оболочка выступает в пустоту
//      не дальше gap (мера CoACD: её чувствует другое тело) и добавляет не больше concavity объёма
//      (мера V-HACD). Разрез проводится через выемку — самую глубокую точку оболочки в пустоте:
//      кольцо так режется на дуги, чашка — на дно и клёпки;
//   3. каждая часть подгоняется к настоящей поверхности: её оболочка строится по точкам исходной
//      сетки, попавшим в область, и по углам вокселей на разрезах;
//   4. соседние части сливаются, пока слитая остаётся почти выпуклой.
//
// Mamou, Ghorbel, «A simple and efficient approach for 3D mesh approximate convex decomposition»,
// ICIP 2009; Wei и др., «Approximate convex decomposition for 3D meshes with collision-aware
// concavity and tree search», ACM TOG 41(4), 2022; Lien, Amato, «Approximate convex decomposition
// of polyhedra», 2007.

#include <kp/math/Vector3.h>

#include <array>
#include <vector>

namespace kp {

// Замкнутая поверхность тела из треугольников.
struct TriangleMesh {
    std::vector<Vector3> vertices;
    std::vector<std::array<int, 3>> triangles; // обход против часовой стрелки, если смотреть снаружи

    // Объём тела: сумма тетраэдров от начала координат к каждому треугольнику.
    Real volume() const {
        Real sum = 0;
        for (const std::array<int, 3>& t : triangles)
            sum += vertices[size_t(t[0])].dot(vertices[size_t(t[1])].cross(vertices[size_t(t[2])])) / 6;
        return sum;
    }
};

struct DecompositionLimits {
    int resolution = 32;          // вокселей вдоль наибольшего размера тела
    int maxParts = 24;            // частей не больше
    int maxHullVertices = 32;     // вершин у части не больше: от их числа зависит цена контакта
    Real gap = Real(0.01);        // насколько оболочка части может выступать в пустоту, доля диагонали тела
    Real concavity = Real(0.015); // сколько лишнего объёма может добавить оболочка части, доля объёма тела
    int maxDepth = 16;            // сколько раз подряд можно резать одну область
};

// Выпуклые части тела: у каждой — её вершины. Пусто, если сетка не охватывает объёма.
std::vector<std::vector<Vector3>> convexParts(const TriangleMesh& solid, const DecompositionLimits& limits = {});

} // namespace kp
