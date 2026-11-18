// Выпуклая оболочка облака точек: алгоритм QuickHull и слияние граней.
//
// Оболочка растёт от начального тетраэдра. У каждой грани есть список точек, которые лежат
// снаружи неё; самая далёкая из них становится новой вершиной: грани, которые она «видит»,
// удаляются, а дыра — её край называется горизонтом — закрывается треугольниками к новой вершине.
// Точки, оказавшиеся внутри, отбрасываются. Barber, Dobkin, Huhdanpaa, «The Quickhull algorithm
// for convex hulls», ACM Transactions on Mathematical Software 22(4), 1996.
//
// QuickHull даёт треугольники. Соседние треугольники, лежащие в одной плоскости, сливаются в
// многоугольную грань: разделяющей оси и отсечению граней (Sat.h, ContactManifold.h) нужны
// настоящие грани тела — у куба их шесть, а не двенадцать треугольников. Образец слияния —
// построитель оболочки в ReactPhysics3D (src/utils/quickhull).

#include <kp/collision/shapes/ConvexHull.h>

#include <algorithm>
#include <map>
#include <stdexcept>
#include <utility>

namespace kp {
namespace {

struct Triangle {
    int v[3];
    Vector3 normal;
    Real offset;              // normal · x = offset на плоскости
    std::vector<int> outside; // точки снаружи этой грани
    bool alive = true;
};

class Builder {
public:
    Builder(const std::vector<Vector3>& points, Real tolerance) : points_(points), tolerance_(tolerance) {}

    void build() {
        startTetrahedron();
        for (size_t f = 0; f < faces_.size(); ++f) // новые грани дописываются в конец и тоже обходятся
            while (faces_[f].alive && !faces_[f].outside.empty()) addVertex(farthest(faces_[f]));
    }

    ConvexHull hull() const;

private:
    Real distance(const Triangle& face, int point) const { return face.normal.dot(points_[size_t(point)]) - face.offset; }

    int addFace(int a, int b, int c) {
        const Vector3 pa = points_[size_t(a)];
        const Vector3 area = (points_[size_t(b)] - pa).cross(points_[size_t(c)] - pa);
        const Vector3 normal = (1 / area.length()) * area;
        faces_.push_back({{a, b, c}, normal, normal.dot(pa), {}, true});
        const int face = int(faces_.size()) - 1;
        for (int k = 0; k < 3; ++k) faceOfEdge_[{faces_.back().v[k], faces_.back().v[(k + 1) % 3]}] = face;
        return face;
    }

    // Точка достаётся первой из граней candidates, снаружи которой лежит; точка внутри оболочки
    // не достаётся никому.
    void assign(int point, const std::vector<int>& candidates) {
        for (int face : candidates) {
            if (distance(faces_[size_t(face)], point) <= tolerance_) continue;
            faces_[size_t(face)].outside.push_back(point);
            return;
        }
    }

    int farthest(const Triangle& face) const {
        int best = face.outside[0];
        for (int point : face.outside)
            if (distance(face, point) > distance(face, best)) best = point;
        return best;
    }

    void startTetrahedron();
    void addVertex(int point);

    const std::vector<Vector3>& points_;
    Real tolerance_;
    std::vector<Triangle> faces_;
    std::map<std::pair<int, int>, int> faceOfEdge_; // направленное ребро -> грань, в обходе которой оно встречается
};

// Начальный тетраэдр: две крайние точки по оси X, самая далёкая от прямой через них и самая
// далёкая от плоскости трёх.
void Builder::startTetrahedron() {
    const int n = int(points_.size());
    int a = 0, b = 0;
    for (int i = 1; i < n; ++i) {
        if (points_[size_t(i)].x < points_[size_t(a)].x) a = i;
        if (points_[size_t(i)].x > points_[size_t(b)].x) b = i;
    }
    const Vector3 pa = points_[size_t(a)], axis = points_[size_t(b)] - pa;
    int c = a;
    for (int i = 0; i < n; ++i)
        if (axis.cross(points_[size_t(i)] - pa).length() > axis.cross(points_[size_t(c)] - pa).length()) c = i;
    const Vector3 plane = axis.cross(points_[size_t(c)] - pa);
    int d = a;
    for (int i = 0; i < n; ++i)
        if (abs(plane.dot(points_[size_t(i)] - pa)) > abs(plane.dot(points_[size_t(d)] - pa))) d = i;
    if (!(abs(plane.dot(points_[size_t(d)] - pa)) > tolerance_ * plane.length()))
        throw std::invalid_argument("выпуклая оболочка: точки лежат в одной плоскости");
    if (plane.dot(points_[size_t(d)] - pa) > 0) std::swap(b, c); // четвёртая вершина — позади грани abc

    const std::vector<int> faces = {addFace(a, b, c), addFace(a, d, b), addFace(b, d, c), addFace(c, d, a)};
    for (int i = 0; i < n; ++i)
        if (i != a && i != b && i != c && i != d) assign(i, faces);
}

void Builder::addVertex(int point) {
    // Грани, которые точка видит, удаляются; их внешние точки ждут новых граней.
    std::vector<int> visible, orphans;
    for (int f = 0; f < int(faces_.size()); ++f)
        if (faces_[size_t(f)].alive && distance(faces_[size_t(f)], point) > tolerance_) visible.push_back(f);
    for (int f : visible) faces_[size_t(f)].alive = false;
    // Горизонт: рёбра видимых граней, за которыми лежит невидимая грань.
    std::vector<std::pair<int, int>> horizon;
    for (int f : visible) {
        Triangle& face = faces_[size_t(f)];
        for (int k = 0; k < 3; ++k) {
            const int from = face.v[k], to = face.v[(k + 1) % 3];
            if (faces_[size_t(faceOfEdge_.at({to, from}))].alive) horizon.push_back({from, to});
        }
        for (int orphan : face.outside)
            if (orphan != point) orphans.push_back(orphan);
        face.outside.clear();
    }
    std::vector<int> created;
    for (const auto& [from, to] : horizon) created.push_back(addFace(from, to, point));
    for (int orphan : orphans) assign(orphan, created);
}

// Треугольники сливаются в грани, грани переводятся в обходы вершин.
ConvexHull Builder::hull() const {
    // Группы копланарных треугольников: два соседа сливаются, если противолежащая вершина каждого
    // лежит в плоскости другого.
    std::vector<int> group(faces_.size());
    for (size_t f = 0; f < group.size(); ++f) group[f] = int(f);
    const auto root = [&](int f) {
        while (group[size_t(f)] != f) f = group[size_t(f)] = group[size_t(group[size_t(f)])];
        return f;
    };
    const auto apex = [&](const Triangle& face, int from, int to) {
        for (int v : face.v)
            if (v != from && v != to) return v;
        return face.v[0];
    };
    for (const auto& [edge, f] : faceOfEdge_) {
        const Triangle& face = faces_[size_t(f)];
        if (!face.alive || edge.first > edge.second) continue;
        const int g = faceOfEdge_.at({edge.second, edge.first});
        const Triangle& other = faces_[size_t(g)];
        if (abs(distance(face, apex(other, edge.first, edge.second))) <= tolerance_ &&
            abs(distance(other, apex(face, edge.first, edge.second))) <= tolerance_)
            group[size_t(std::max(root(f), root(g)))] = std::min(root(f), root(g));
    }

    // Край группы: рёбра, за которыми лежит другая группа. next — следующая вершина обхода,
    // across — группа за ребром, выходящим из вершины.
    std::map<int, std::map<int, std::pair<int, int>>> border; // группа -> вершина -> (следующая, группа за ребром)
    for (const auto& [edge, f] : faceOfEdge_) {
        if (!faces_[size_t(f)].alive) continue;
        const int here = root(f), there = root(faceOfEdge_.at({edge.second, edge.first}));
        if (here != there) border[here][edge.first] = {edge.second, there};
    }

    std::vector<int> index(points_.size(), -1); // новый номер вершины
    std::vector<Vector3> vertices;
    std::vector<std::vector<int>> loops;
    for (const auto& [face, next] : border) {
        std::vector<int> loop;
        for (int v = next.begin()->first; loop.size() < next.size(); v = next.at(v).first) {
            // Вершина посреди прямого ребра — та, по обе стороны от которой за краем лежит одна и
            // та же соседняя грань, — настоящей вершиной не является.
            int previous = -1;
            for (const auto& [from, to] : next)
                if (to.first == v) previous = from;
            const bool onStraightEdge = next.at(previous).second == next.at(v).second;
            if (!onStraightEdge) loop.push_back(v);
            if (next.at(v).first == next.begin()->first) break;
        }
        for (int& v : loop) {
            if (index[size_t(v)] < 0) index[size_t(v)] = int(vertices.size()), vertices.push_back(points_[size_t(v)]);
            v = index[size_t(v)];
        }
        loops.push_back(loop);
    }
    return ConvexHull(std::move(vertices), loops);
}

} // namespace

ConvexHull ConvexHull::fromPoints(const std::vector<Vector3>& points, Real tolerance) {
    if (points.size() < 4) throw std::invalid_argument("выпуклая оболочка: нужно не меньше четырёх точек");
    Vector3 lower = points[0], upper = points[0];
    for (Vector3 p : points) {
        lower = {std::min(lower.x, p.x), std::min(lower.y, p.y), std::min(lower.z, p.z)};
        upper = {std::max(upper.x, p.x), std::max(upper.y, p.y), std::max(upper.z, p.z)};
    }
    const Vector3 size = upper - lower;
    Builder builder(points, tolerance > 0 ? tolerance : Real(1e-9) * std::max({size.x, size.y, size.z}));
    builder.build();
    return builder.hull();
}

} // namespace kp
