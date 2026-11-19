#pragma once
// Многогранник, перенесённый в общую систему координат: вершины и нормали граней уже повёрнуты.
// Им пользуются разделяющая ось (Sat.cpp) и построение точек контакта (HullVsHullAlgorithm.cpp).

#include <collision/shapes/ConvexHull.h>
#include <math/Transform.h>

#include <vector>

namespace kp {

struct TransformedHull {
    const ConvexHull& shape;
    std::vector<Vector3> vertices, normals;
    Vector3 centre;

    TransformedHull(const ConvexHull& hull, const Transform& pose) : shape(hull), centre(pose.position) {
        for (Vector3 v : hull.vertices()) vertices.push_back(pose.point(v));
        for (const ConvexHull::Face& f : hull.faces()) normals.push_back(pose.direction(f.normal));
    }
    // Самая далёкая вершина в направлении d.
    Vector3 support(Vector3 d) const {
        size_t best = 0;
        for (size_t i = 1; i < vertices.size(); ++i)
            if (vertices[i].dot(d) > vertices[best].dot(d)) best = i;
        return vertices[best];
    }
    // Любая точка грани: её первая вершина.
    Vector3 pointOnFace(int face) const { return vertices[size_t(shape.faces()[size_t(face)].loop[0])]; }
    const std::vector<int>& loop(int face) const { return shape.faces()[size_t(face)].loop; }
};

} // namespace kp
