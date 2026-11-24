#include <collision/shapes/Sphere.h>
#include <body/CompoundBodyBuilder.h>

#include <body/MassFrame.h>
#include <body/PrincipalAxes.h>

#include <math/math_functions.h>

#include <algorithm>

namespace kp {

constexpr size_t kTreeParts = 8; // с какого числа частей составному телу строится дерево

// Масса составного тела — сумма масс частей, центр масс — их взвешенное среднее. Тензор инерции
// части относительно общего центра масс — её собственный тензор R diag(I) Rᵀ плюс перенос по
// теореме Штейнера m (|d|² E − d dᵀ), где d — от общего центра масс до центра масс части. Сумма
// приводится к главным осям: они и становятся осями тела.
namespace {

// Начало осей, главные оси и массы составного тела по его частям. У неподвижного тела масс нет;
// началом его осей служит среднее положение частей.
void placeFrame(Body& body, const std::vector<Body>& pieces, bool fixed) {
    const auto massOf = [&](const Body& piece) { return fixed ? Real(1) : 1 / piece.mass.inverseMass; };
    Real mass = 0;
    Vector3 moment;
    for (const Body& piece : pieces) mass += massOf(piece), moment += massOf(piece) * piece.position;
    body.position = (1 / mass) * moment;
    if (fixed) return;
    Real inertia[3][3] = {};
    for (const Body& piece : pieces) {
        const Vector3 d = piece.position - body.position;
        const Real offset[3] = {d.x, d.y, d.z}, own[3] = {1 / piece.mass.inverseInertia.x, 1 / piece.mass.inverseInertia.y, 1 / piece.mass.inverseInertia.z};
        for (int k = 0; k < 3; ++k) {
            const Vector3 axis = piece.orientation.rotate({Real(k == 0), Real(k == 1), Real(k == 2)});
            const Real e[3] = {axis.x, axis.y, axis.z};
            for (int r = 0; r < 3; ++r)
                for (int s = 0; s < 3; ++s) inertia[r][s] += own[k] * e[r] * e[s];
        }
        for (int r = 0; r < 3; ++r)
            for (int s = 0; s < 3; ++s) inertia[r][s] += massOf(piece) * ((r == s ? d.dot(d) : Real(0)) - offset[r] * offset[s]);
    }
    const PrincipalAxes axes = PrincipalAxes::of(inertia);
    body.orientation = axes.orientation;
    body.mass = {1 / mass, {1 / axes.moments.x, 1 / axes.moments.y, 1 / axes.moments.z}};
}

// Дерево брусков частей: брусок части — её точные габариты вдоль осей тела.
std::shared_ptr<DynamicTree> treeOf(const std::vector<Body::Part>& parts) {
    auto tree = std::make_shared<DynamicTree>();
    for (size_t i = 0; i < parts.size(); ++i) {
        const Body::Part& part = parts[i];
        const auto reach = [&](Vector3 axis) {
            const Vector3 local = part.placement.localDirection(axis);
            return axis.dot(part.placement.point(part.shape->support(local))) + part.shape->radius;
        };
        tree->insert({{-reach({-1, 0, 0}), -reach({0, -1, 0}), -reach({0, 0, -1})}, {reach({1, 0, 0}), reach({0, 1, 0}), reach({0, 0, 1})}}, int(i));
    }
    return tree;
}

// На торце клина наружная нормаль направлена по касательной к окружности.
// QuickHull нумерует грани каждого клина независимо. Ищем грань в собственных осях формы.
int sectorEnd(const Body& wedge, Vector3 outward) {
    const Vector3 local = wedge.pose().localDirection(outward);
    const auto& faces = wedge.shape->hull()->faces();
    for (size_t i = 0; i < faces.size(); ++i)
        if (faces[i].normal.dot(local) >= 1 - 64 * kEpsilon) return int(i);
    return -1;
}

// Соседние клинья построены из одного профиля: конец предыдущего и начало следующего
// содержат одни и те же точки around(point, seam). Отмечаем только подтверждённую пару.
void markSectorEnds(std::vector<Body>& pieces, size_t first, int segments) {
    struct Ends { int begin = -1, end = -1; };
    std::vector<Ends> ends(static_cast<size_t>(segments));
    for (int seam = 0; seam < segments; ++seam) {
        const size_t prev = size_t(seam == 0 ? segments - 1 : seam - 1), next = size_t(seam);
        const SinCos turn = sinCos(2 * kPi * Real(seam) / Real(segments));
        const Vector3 tangent{-turn.sin, 0, turn.cos};
        const int left = sectorEnd(pieces[first + prev], tangent);
        const int right = sectorEnd(pieces[first + next], -tangent);
        if (left >= 0 && right >= 0) ends[prev].end = left, ends[next].begin = right;
    }
    for (size_t i = 0; i < ends.size(); ++i) {
        if (ends[i].begin < 0 && ends[i].end < 0) continue;
        Body& piece = pieces[first + i];
        auto hull = std::make_shared<ConvexHull>(*piece.shape->hull());
        if (ends[i].begin >= 0) hull->markInternalFace(ends[i].begin);
        if (ends[i].end >= 0) hull->markInternalFace(ends[i].end);
        piece.shape = std::move(hull);
    }
}

} // namespace

Body CompoundBodyBuilder::compound(const std::vector<Body>& pieces) {
    Body body;
    if (pieces.empty()) return body;
    body.friction = pieces[0].friction, body.restitution = pieces[0].restitution;
    placeFrame(body, pieces, std::any_of(pieces.begin(), pieces.end(), [](const Body& piece) { return piece.isStatic(); }));
    // Части: каждая форма — в осях нового тела. Составная часть входит своими частями.
    auto parts = std::make_shared<std::vector<Body::Part>>();
    Real radius = 0;
    for (const Body& piece : pieces) {
        for (int i = 0; i < piece.partCount(); ++i) {
            const Body part = piece.part(i);
            if (!part.shape) continue;
            parts->push_back({part.shape, frameOf(body).relative(part.pose())});
            radius = std::max(radius, parts->back().placement.position.length() + part.shape->boundingRadius());
        }
    }
    body.shape = std::make_shared<Sphere>(radius);
    if (parts->size() >= kTreeParts) body.partTree = treeOf(*parts);
    body.parts = std::move(parts);
    return body;
}

Body CompoundBodyBuilder::revolved(const std::vector<std::vector<std::pair<Real, Real>>>& profiles, int segments, Real density, Vector3 position) {
    const auto around = [&](const std::pair<Real, Real>& point, int segment) {
        const SinCos turn = sinCos(2 * kPi * Real(segment % segments) / Real(segments));
        return Vector3{point.first * turn.cos, point.second, point.first * turn.sin};
    };
    std::vector<Body> pieces;
    for (const std::vector<std::pair<Real, Real>>& profile : profiles) {
        // Клинья: оболочка двух соседних положений многоугольника.
        std::vector<std::vector<Vector3>> wedges{size_t(segments)};
        std::vector<Vector3> whole;
        Real volume = 0;
        for (int s = 0; s < segments; ++s) {
            for (const std::pair<Real, Real>& point : profile) wedges[size_t(s)].push_back(around(point, s)), wedges[size_t(s)].push_back(around(point, s + 1));
            for (const std::pair<Real, Real>& point : profile) whole.push_back(around(point, s));
            volume += MassFrame::of(ConvexHull::fromPoints(wedges[size_t(s)]), 0).volume;
        }
        // Вращение многоугольника выпукло целиком, если его оболочка не больше суммы клиньев.
        if (MassFrame::of(ConvexHull::fromPoints(whole), 0).volume <= volume * (1 + Real(1e-9))) {
            pieces.push_back(Body::hull(whole, density, position));
            continue;
        }
        const size_t first = pieces.size();
        for (const std::vector<Vector3>& wedge : wedges) pieces.push_back(Body::hull(wedge, density, position));
        markSectorEnds(pieces, first, segments);
    }
    return compound(pieces);
}

Body CompoundBodyBuilder::torus(Real ring, Real tube, Real density, Vector3 position, int around, int across) {
    std::vector<std::pair<Real, Real>> circle;
    for (int s = 0; s < across; ++s) {
        const SinCos turn = sinCos(2 * kPi * Real(s) / Real(across));
        circle.push_back({ring + tube * turn.cos, tube * turn.sin});
    }
    return revolved({circle}, around, density, position);
}

Body CompoundBodyBuilder::bucket(Real bottomRadius, Real topRadius, Real height, Real wall, Real density, Vector3 position, int segments) {
    // Тот же профиль, что у bucketMesh, разрезанный на дно и стенку.
    const Real innerBottom = bottomRadius - wall + (topRadius - bottomRadius) * wall / height;
    return revolved({{{0, 0}, {bottomRadius, 0}, {innerBottom, wall}, {0, wall}}, {{bottomRadius, 0}, {topRadius, height}, {topRadius - wall, height}, {innerBottom, wall}}}, segments, density, position);
}

Body CompoundBodyBuilder::ground(const TriangleMesh& surface, Real thickness) {
    std::vector<Body> pieces;
    for (const std::array<int, 3>& t : surface.triangles) {
        const Vector3 a = surface.vertices[size_t(t[0])], b = surface.vertices[size_t(t[1])], c = surface.vertices[size_t(t[2])];
        const Vector3 normal = (b - a).cross(c - a);
        if (normal.length() == 0) continue; // вырожденный треугольник
        const Vector3 inward = (-thickness / normal.length()) * normal;
        Body piece = Body::hull({a, b, c, a + inward, b + inward, c + inward}, 0);
        // Наружная грань призмы — сам треугольник: грань, нормаль которой ближе всех к его нормали.
        auto prism = std::make_shared<ConvexHull>(*static_cast<const ConvexHull*>(piece.shape.get()));
        const Vector3 outward = piece.pose().localDirection(normal);
        for (size_t f = 0; f < prism->faces().size(); ++f)
            if (prism->outerFace < 0 || prism->faces()[f].normal.dot(outward) > prism->faces()[size_t(prism->outerFace)].normal.dot(outward)) prism->outerFace = int(f);
        piece.shape = prism;
        pieces.push_back(piece);
    }
    return compound(pieces);
}

Body CompoundBodyBuilder::solid(const TriangleMesh& surface, Real density, Vector3 position, const DecompositionLimits& limits) {
    const std::vector<std::vector<Vector3>> parts = convexParts(surface, limits);
    Real volume = 0;
    for (const std::vector<Vector3>& points : parts) volume += MassFrame::of(ConvexHull::fromPoints(points), 0).volume;
    const Real scaled = volume > 0 ? density * abs(surface.volume()) / volume : density;
    std::vector<Body> pieces;
    for (const std::vector<Vector3>& points : parts) pieces.push_back(Body::hull(points, scaled, position));
    return compound(pieces);
}

} // namespace kp
