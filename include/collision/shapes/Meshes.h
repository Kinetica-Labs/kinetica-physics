#pragma once
// Сетки поверхностей: тела вращения — заготовки для невыпуклых тел (ConvexDecomposition.h) — и рельеф
// по карте высот для неподвижного мира (CompoundBodyBuilder::ground).

#include <collision/shapes/ConvexDecomposition.h>

#include <utility>

namespace kp {

// Тело вращения вокруг оси Y. profile — замкнутая ломаная в полуплоскости (r, y), r >= 0; точки на
// оси (r = 0) допустимы. segments — число граней по окружности.
TriangleMesh lathe(const std::vector<std::pair<Real, Real>>& profile, int segments);

// Тор вокруг оси Y: ring — радиус осевой окружности, tube — радиус трубки.
TriangleMesh torusMesh(Real ring, Real tube, int around = 24, int across = 12);

// Ведро донышком вниз, ось Y, дно на высоте ноль: bottomRadius и topRadius — наружные радиусы дна и
// кромки, wall — толщина стенки и дна.
TriangleMesh bucketMesh(Real bottomRadius, Real topRadius, Real height, Real wall, int segments = 24);

// Рельеф по карте высот: heights — высоты узлов сетки columns × rows, строка за строкой (вдоль x, строки —
// вдоль z), шаг сетки cell, м; сетка стоит вокруг начала координат. Клетка — два треугольника, наружу —
// вверх. Сетка не замкнута: это поверхность для CompoundBodyBuilder::ground.
TriangleMesh heightfieldMesh(const std::vector<Real>& heights, int columns, int rows, Real cell);

} // namespace kp
