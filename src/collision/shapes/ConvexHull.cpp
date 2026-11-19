#include <collision/shapes/ConvexHull.h>

#include <collision/shapes/Flat.h>
#include <collision/shapes/Outline.h>

#include <algorithm>
#include <map>
#include <utility>

namespace kp {

ConvexHull::ConvexHull(std::vector<Vector3> vertices, const std::vector<std::vector<int>>& loops) : vertices_(std::move(vertices)) {
    // Ребро tail -> head встречается в обходе одной грани, а head -> tail — в обходе соседней.
    for (Vector3 v : vertices_) extent_ = std::max(extent_, v.length());
    std::map<std::pair<int, int>, int> faceOfDirectedEdge;
    for (const std::vector<int>& loop : loops) {
        // Нормаль по Ньюэллу: сумма векторных произведений соседних вершин обхода. Для плоского
        // выпуклого многоугольника это удвоенная площадь, направленная наружу.
        Vector3 areaVector;
        for (size_t i = 0; i < loop.size(); ++i)
            areaVector += vertices_[size_t(loop[i])].cross(vertices_[size_t(loop[(i + 1) % loop.size()])]);
        const Vector3 normal = (1 / areaVector.length()) * areaVector;
        const int face = int(faces_.size());
        faces_.push_back({normal, normal.dot(vertices_[size_t(loop[0])]), loop});
        for (size_t i = 0; i < loop.size(); ++i) faceOfDirectedEdge[{loop[i], loop[(i + 1) % loop.size()]}] = face;
    }
    // Каждое ребро записывается один раз — в том направлении, где номер хвоста меньше.
    for (const auto& [directed, face] : faceOfDirectedEdge)
        if (directed.first < directed.second)
            edges_.push_back({directed.first, directed.second, face, faceOfDirectedEdge.at({directed.second, directed.first})});
}

ConvexHull ConvexHull::box(Vector3 h) {
    // Номер вершины — три бита: x, y, z; единица означает положительную сторону.
    std::vector<Vector3> vertices;
    for (int i = 0; i < 8; ++i) vertices.push_back({i & 1 ? h.x : -h.x, i & 2 ? h.y : -h.y, i & 4 ? h.z : -h.z});
    return ConvexHull(std::move(vertices), {{0, 4, 6, 2}, {1, 3, 7, 5},   // −x, +x
                                          {0, 1, 5, 4}, {2, 6, 7, 3},   // −y, +y
                                          {0, 2, 3, 1}, {4, 5, 7, 6}}); // −z, +z
}

// Перебор вершин. При равенстве остаётся вершина с меньшим номером — выбор не зависит ни от чего,
// кроме самих чисел.
Vector3 ConvexHull::support(Vector3 direction) const {
    size_t best = 0;
    Real bestProjection = vertices_[0].dot(direction);
    for (size_t i = 1; i < vertices_.size(); ++i) {
        const Real projection = vertices_[i].dot(direction);
        if (projection > bestProjection) best = i, bestProjection = projection;
    }
    return vertices_[best];
}

Flat ConvexHull::faceFlat(const Transform& pose, int face) const {
    Flat flat;
    flat.normal = pose.direction(faces_[size_t(face)].normal);
    for (int vertex : faces_[size_t(face)].loop) flat.corners.push_back(pose.point(vertices_[size_t(vertex)]));
    flat.origin = flat.corners[0];
    return flat;
}

std::optional<Flat> ConvexHull::flatTowards(const Transform& pose, Vector3 towards) const {
    const Vector3 local = pose.localDirection(towards);
    int best = 0;
    for (int f = 1; f < int(faces_.size()); ++f)
        if (faces_[size_t(f)].normal.dot(local) > faces_[size_t(best)].normal.dot(local)) best = f;
    return faceFlat(pose, best);
}

Outline ConvexHull::outlineOver(const Transform& pose, const Flat& flat) const {
    Outline outline;
    outline.rounding = radius;
    // Грань при самой низкой вершине, сильнее всех смотрящая навстречу.
    const Vector3 local = pose.localDirection(flat.normal);
    int lowest = 0, best = -1;
    for (int v = 1; v < int(vertices_.size()); ++v)
        if (vertices_[size_t(v)].dot(local) < vertices_[size_t(lowest)].dot(local)) lowest = v;
    for (int f = 0; f < int(faces_.size()); ++f) {
        const Face& face = faces_[size_t(f)];
        if (std::find(face.loop.begin(), face.loop.end(), lowest) == face.loop.end()) continue;
        if (best < 0 || face.normal.dot(local) < faces_[size_t(best)].normal.dot(local)) best = f;
    }
    const Flat face = faceFlat(pose, best);
    outline.kind = Outline::Kind::Loop, outline.points = face.corners, outline.normal = face.normal;
    return outline;
}

} // namespace kp
