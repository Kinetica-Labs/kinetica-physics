#include <collision/PairFinder.h>
#include <collision/shapes/Capsule.h>
#include <collision/shapes/Cylinder.h>
#include <debug/DebugGeometry.h>

#include <cstdio>

namespace kp {
namespace {

// Окружность из 24 отрезков в плоскости единичных векторов u и v. Каждая следующая точка —
// предыдущая, повёрнутая на 15°; косинус и синус этого угла записаны числами, поэтому
// библиотечные функции, которые на разных машинах округляют по-разному, не нужны.
void circle(DebugGeometry& geometry, Vector3 centre, Vector3 u, Vector3 v, Real radius, DebugKind kind) {
    const Real cos15 = Real(0.96592582628906829), sin15 = Real(0.25881904510252076);
    Real x = radius, y = 0;
    for (int k = 0; k < 24; ++k) {
        const Real nextX = cos15 * x - sin15 * y, nextY = sin15 * x + cos15 * y;
        geometry.lines.push_back({centre + x * u + y * v, centre + nextX * u + nextY * v, kind});
        x = nextX;
        y = nextY;
    }
}

// Шар: три окружности во взаимно перпендикулярных плоскостях осей тела.
void ball(DebugGeometry& geometry, const Transform& pose, Vector3 localCentre, Real radius, DebugKind kind) {
    const Vector3 centre = pose.point(localCentre);
    const Vector3 x = pose.direction({1, 0, 0}), y = pose.direction({0, 1, 0}), z = pose.direction({0, 0, 1});
    circle(geometry, centre, x, y, radius, kind);
    circle(geometry, centre, y, z, radius, kind);
    circle(geometry, centre, z, x, radius, kind);
}

template <class... Values> std::string line(const char* format, Values... values) {
    char text[256];
    std::snprintf(text, sizeof text, format, values...);
    return text;
}

} // namespace

DebugGeometry& DebugGeometry::add(const ConvexShape& shape, const Transform& pose, DebugKind kind) {
    if (const auto* hull = dynamic_cast<const ConvexHull*>(&shape)) {
        for (const ConvexHull::Edge& edge : hull->edges())
            lines.push_back({pose.point(hull->vertices()[size_t(edge.tail)]), pose.point(hull->vertices()[size_t(edge.head)]), kind});
    } else if (const auto* capsule = dynamic_cast<const Capsule*>(&shape)) {
        const Real r = shape.radius, h = capsule->halfLength;
        ball(*this, pose, {0, -h, 0}, r, kind);
        ball(*this, pose, {0, h, 0}, r, kind);
        for (Vector3 side : {Vector3{r, 0, 0}, Vector3{-r, 0, 0}, Vector3{0, 0, r}, Vector3{0, 0, -r}})
            lines.push_back({pose.point(side - Vector3{0, h, 0}), pose.point(side + Vector3{0, h, 0}), kind});
    } else if (const auto* cylinder = dynamic_cast<const Cylinder*>(&shape)) {
        const Real r = cylinder->coreRadius + shape.radius, h = cylinder->coreHalfHeight + shape.radius;
        const Vector3 x = pose.direction({1, 0, 0}), z = pose.direction({0, 0, 1});
        circle(*this, pose.point({0, -h, 0}), z, x, r, kind);
        circle(*this, pose.point({0, h, 0}), z, x, r, kind);
        for (Vector3 side : {Vector3{r, 0, 0}, Vector3{-r, 0, 0}, Vector3{0, 0, r}, Vector3{0, 0, -r}})
            lines.push_back({pose.point(side - Vector3{0, h, 0}), pose.point(side + Vector3{0, h, 0}), kind});
    } else {
        ball(*this, pose, {}, shape.radius, kind);
    }
    return *this;
}

DebugGeometry& DebugGeometry::add(const ContactManifold& manifold) {
    for (int k = 0; k < manifold.count; ++k) {
        const Vector3 point = manifold.points[k].onA;
        points.push_back({point, DebugKind::ContactPoint});
        lines.push_back({point, point + normalLength * manifold.normal, DebugKind::ContactNormal});
    }
    return *this;
}

DebugGeometry& DebugGeometry::add(const ClosestPoints& closest) {
    points.push_back({closest.onA, DebugKind::ClosestPoints});
    points.push_back({closest.onB, DebugKind::ClosestPoints});
    lines.push_back({closest.onA, closest.onB, DebugKind::ClosestPoints});
    return *this;
}

DebugGeometry& DebugGeometry::add(const DynamicTree& tree) {
    for (const DynamicTree::Node& node : tree.nodes()) {
        if (node.leaf()) continue;
        const Vector3 corner[2] = {node.bounds.lower, node.bounds.upper};
        // Двенадцать рёбер бруска: от каждой из четырёх вершин с чётным числом «верхних» координат
        // — по ребру вдоль каждой оси.
        for (int vertex : {0, 3, 5, 6}) {
            const Vector3 from{corner[vertex & 1].x, corner[vertex >> 1 & 1].y, corner[vertex >> 2 & 1].z};
            lines.push_back({from, {corner[~vertex & 1].x, from.y, from.z}, DebugKind::TreeNode});
            lines.push_back({from, {from.x, corner[~vertex >> 1 & 1].y, from.z}, DebugKind::TreeNode});
            lines.push_back({from, {from.x, from.y, corner[~vertex >> 2 & 1].z}, DebugKind::TreeNode});
        }
    }
    return *this;
}

DebugGeometry debugGeometry(const World& world, bool withTree) {
    DebugGeometry geometry;
    if (withTree) geometry.add(PairFinder(world.bodies(), world.gravity).tree(0));
    for (const Body& body : world.bodies()) {
        for (int i = 0; body.shape && i < body.partCount(); ++i) {
            const Body part = body.part(i);
            geometry.add(*part.shape, part.pose(), body.isStatic() ? DebugKind::StaticShape : DebugKind::Shape);
        }
    }
    // Кривые сочленений по траектории — ломаной, шестнадцать звеньев на отрезок сплайна.
    for (size_t j = 0; j < world.joints().size(); ++j) {
        const PathJoint* rail = dynamic_cast<const PathJoint*>(&world.joints()[j]);
        if (!rail) continue;
        const Transform seat = world.bodies()[size_t(rail->a)].pose();
        for (int k = 0; k < 16 * int(rail->path().span()); ++k)
            geometry.lines.push_back({seat.point(rail->path().point(Real(k) / 16)), seat.point(rail->path().point(Real(k + 1) / 16)), DebugKind::StaticShape});
    }
    for (const ContactPair& pair : PairFinder(world.bodies(), world.gravity).pairs(0))
        if (pair.inContact()) geometry.add(pair.manifold);
    return geometry;
}

std::string debugText(const World& world) {
    std::string text;
    int index = 0;
    for (const Body& body : world.bodies()) {
        const Vector3 p = body.position, v = body.linearVelocity(), w = body.angularVelocity();
        text += body.isStatic()
                    ? line("тело %d: неподвижное, центр (%.3f, %.3f, %.3f) м\n", index, p.x, p.y, p.z)
                    : line("тело %d: центр (%.3f, %.3f, %.3f) м, скорость (%.3f, %.3f, %.3f) м/с, вращение (%.3f, %.3f, %.3f) рад/с\n",
                           index, p.x, p.y, p.z, v.x, v.y, v.z, w.x, w.y, w.z);
        ++index;
    }
    for (const ContactPair& pair : PairFinder(world.bodies(), world.gravity).pairs(0)) {
        if (!pair.inContact()) continue;
        const Vector3 n = pair.manifold.normal;
        text += line("контакт тел %d и %d: точек %d, нормаль (%.3f, %.3f, %.3f), зазор %.2e м\n", pair.a, pair.b,
                     pair.manifold.count, n.x, n.y, n.z, pair.distance);
    }
    text += line("дерево поиска пар: высота %d при %zu телах\n", PairFinder(world.bodies(), world.gravity).tree(0).height(), world.bodies().size());
    return text + line("последний шаг: отрезков %d, проходов решателя %d\n", world.report().segments, world.report().solverIterations);
}

} // namespace kp
