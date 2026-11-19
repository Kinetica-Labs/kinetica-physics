#include <collision/shapes/Meshes.h>

#include <math/math_functions.h>

namespace kp {

TriangleMesh lathe(const std::vector<std::pair<Real, Real>>& profile, int segments) {
    TriangleMesh mesh;
    // Точка профиля на оси — одна вершина, остальные — по вершине на каждую грань окружности.
    std::vector<int> first(profile.size());
    for (size_t p = 0; p < profile.size(); ++p) {
        first[p] = int(mesh.vertices.size());
        const auto [r, y] = profile[p];
        if (r == 0) {
            mesh.vertices.push_back({0, y, 0});
            continue;
        }
        for (int s = 0; s < segments; ++s) {
            const SinCos turn = sinCos(2 * kPi * Real(s) / Real(segments));
            mesh.vertices.push_back({r * turn.cos, y, r * turn.sin});
        }
    }
    const auto vertex = [&](size_t p, int s) { return profile[p].first == 0 ? first[p] : first[p] + s % segments; };
    for (size_t p = 0; p < profile.size(); ++p) {
        const size_t q = (p + 1) % profile.size();
        const bool onAxisP = profile[p].first == 0, onAxisQ = profile[q].first == 0;
        if (onAxisP && onAxisQ) continue;
        for (int s = 0; s < segments; ++s) {
            if (!onAxisQ) mesh.triangles.push_back({vertex(p, s), vertex(q, s), vertex(q, s + 1)});
            if (!onAxisP) mesh.triangles.push_back({vertex(p, s), vertex(q, s + 1), vertex(p, s + 1)});
        }
    }
    // Обход граней зависит от того, в какую сторону обойдён профиль; наружу — тот, при котором
    // объём положителен.
    if (mesh.volume() < 0)
        for (std::array<int, 3>& t : mesh.triangles) std::swap(t[1], t[2]);
    return mesh;
}

TriangleMesh torusMesh(Real ring, Real tube, int around, int across) {
    std::vector<std::pair<Real, Real>> circle;
    for (int s = 0; s < across; ++s) {
        const SinCos turn = sinCos(2 * kPi * Real(s) / Real(across));
        circle.push_back({ring + tube * turn.cos, tube * turn.sin});
    }
    return lathe(circle, around);
}

TriangleMesh bucketMesh(Real bottomRadius, Real topRadius, Real height, Real wall, int segments) {
    // Внутренний радиус у дна — там, где внутренняя стенка, параллельная наружной, встречает дно.
    const Real innerBottom = bottomRadius - wall + (topRadius - bottomRadius) * wall / height;
    return lathe({{0, 0}, {bottomRadius, 0}, {topRadius, height}, {topRadius - wall, height}, {innerBottom, wall}, {0, wall}}, segments);
}

TriangleMesh heightfieldMesh(const std::vector<Real>& heights, int columns, int rows, Real cell) {
    TriangleMesh mesh;
    for (int k = 0; k < rows; ++k)
        for (int i = 0; i < columns; ++i)
            mesh.vertices.push_back({(Real(i) - Real(columns - 1) / 2) * cell, heights[size_t(k * columns + i)], (Real(k) - Real(rows - 1) / 2) * cell});
    for (int k = 0; k + 1 < rows; ++k)
        for (int i = 0; i + 1 < columns; ++i) {
            const int v = k * columns + i;
            mesh.triangles.push_back({v, v + columns, v + columns + 1}), mesh.triangles.push_back({v, v + columns + 1, v + 1});
        }
    return mesh;
}

} // namespace kp
