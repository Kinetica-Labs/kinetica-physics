#include <collision/narrowphase/HullVsHullAlgorithm.h>

#include <collision/narrowphase/ContactCandidate.h>
#include <collision/narrowphase/Sat.h>
#include <collision/narrowphase/TransformedHull.h>

#include <algorithm>
#include <limits>
#include <vector>

namespace kp {
namespace {

// Номер точки: вершина обрезаемой грани — это она сама; точка пересечения — ребро опорной грани
// и ребро обрезаемой, на котором она возникла.
std::uint32_t vertexId(int vertex) { return std::uint32_t(vertex); }
// Поля по 8 бит: вершин у грани и у тела пока не больше 256.
std::uint32_t crossingId(int referenceEdge, int tail, int head) {
    return 0x80000000u | std::uint32_t(referenceEdge) << 16 | std::uint32_t(tail) << 8 | std::uint32_t(head);
}

// Обрезает многоугольник полуплоскостью side · (x − onPlane) <= 0 (Сазерленд — Ходжман):
// вершины внутри остаются, на каждом ребре, пересекающем границу, появляется новая вершина.
using VertexNumbers = SmallVector<int, 16>;

ContactCandidates clip(const ContactCandidates& polygon, const VertexNumbers& vertexOf, Vector3 side, Vector3 onPlane, Real boundary, int referenceEdge, VertexNumbers& vertexOfClipped) {
    ContactCandidates kept;
    vertexOfClipped.clear();
    const Real sideLength = side.length();
    for (size_t i = 0; i < polygon.size(); ++i) {
        const size_t j = (i + 1) % polygon.size();
        // side не единичный: расстояния до границы делятся на его длину.
        const Real di = side.dot(polygon[i].point - onPlane) / sideLength, dj = side.dot(polygon[j].point - onPlane) / sideLength;
        if (di <= boundary) kept.push_back(polygon[i]), vertexOfClipped.push_back(vertexOf[i]);
        if ((di < -boundary && dj > boundary) || (di > boundary && dj < -boundary)) {
            const Real t = di / (di - dj);
            kept.push_back({polygon[i].point + t * (polygon[j].point - polygon[i].point), 0,
                            crossingId(referenceEdge, vertexOf[i], vertexOf[j])});
            // Новая точка наследует ту вершину ребра, которая осталась снаружи: тогда кусок
            // ребра по эту сторону границы по-прежнему называется парой исходных вершин.
            vertexOfClipped.push_back(di < 0 ? vertexOf[j] : vertexOf[i]);
        }
    }
    return kept;
}

// Грань, которой тело обращено к плоскости с нормалью n: из граней при самой глубокой вершине —
// та, что сильнее всех смотрит навстречу n. Самая глубокая вершина обязана попасть в контакт, а
// грань, которая просто сильнее всех смотрит навстречу, может её не содержать: у острой вершины
// пирамиды все грани круты, и какая-нибудь грань в стороне оказывается «более встречной». У бруска
// оба выбора совпадают.
int facingFace(const TransformedHull& body, Vector3 n) {
    int deepest = 0;
    Real least = body.vertices[0].dot(n);
    for (int i = 1; i < int(body.vertices.size()); ++i)
        if (const Real along = body.vertices[size_t(i)].dot(n); along < least) least = along, deepest = i;
    int best = -1;
    for (int face = 0; face < int(body.normals.size()); ++face) {
        const std::vector<int>& loop = body.loop(face);
        if (std::find(loop.begin(), loop.end(), deepest) == loop.end()) continue;
        if (best < 0 || body.normals[size_t(face)].dot(n) < body.normals[size_t(best)].dot(n)) best = face;
    }
    return best;
}

// Самое короткое ребро грани с контуром loop.
Real shortestEdge(const TransformedHull& body, const std::vector<int>& loop) {
    Real shortest = std::numeric_limits<Real>::infinity();
    for (size_t k = 0; k < loop.size(); ++k) shortest = std::min(shortest, (body.vertices[size_t(loop[(k + 1) % loop.size()])] - body.vertices[size_t(loop[k])]).length());
    return shortest;
}

// У тела с мелкими гранями к опорной грани близки и вершины соседних граней: конус, катящийся по
// полу, касается его сначала одной боковой гранью, а уже подходит вершина следующей. Такие вершины
// тоже входят в контакт — иначе о них не знают ни поиск времени удара, ни раздвигание, и тело
// постепенно уходит в опору. У бруска соседних вершин рядом нет, и ничего не меняется.
void addNearVertices(const TransformedHull& reference, int face, const TransformedHull& incident, const std::vector<int>& incidentLoop, Real margin, Real boundary, ContactCandidates& near) {
    const Vector3 n = reference.normals[size_t(face)];
    const std::vector<int>& loop = reference.loop(face);
    for (int vertex = 0; vertex < int(incident.vertices.size()); ++vertex) {
        if (std::find(incidentLoop.begin(), incidentLoop.end(), vertex) != incidentLoop.end()) continue;
        const Vector3 point = incident.vertices[size_t(vertex)];
        const Real separation = n.dot(point - reference.pointOnFace(face));
        if (separation > margin) continue;
        bool inside = true;
        for (size_t k = 0; k < loop.size() && inside; ++k) {
            const Vector3 tail = reference.vertices[size_t(loop[k])], side = (reference.vertices[size_t(loop[(k + 1) % loop.size()])] - tail).cross(n);
            inside = side.dot(point - tail) <= boundary * side.length();
        }
        if (inside) near.push_back({point, separation, vertexId(vertex)});
    }
}

// Контакт по грани тела reference: обращённая к ней грань тела incident обрезается по её контуру.
ContactCandidates clipAgainstFace(const TransformedHull& reference, int face, const TransformedHull& incident, Real margin, Real touching, Real boundary) {
    const Vector3 n = reference.normals[size_t(face)];
    const std::vector<int>& incidentLoop = incident.loop(facingFace(incident, n));
    ContactCandidates polygon;
    VertexNumbers vertexOf, scratch;
    for (int vertex : incidentLoop) polygon.push_back({incident.vertices[size_t(vertex)], 0, vertexId(vertex)}), vertexOf.push_back(vertex);

    const std::vector<int>& loop = reference.loop(face);
    // Запас на границе — не больше сотой доли самого короткого ребра опорной грани: миллиметр,
    // безобидный для куба, у клина стенки ведра сдвигал опору, и вёдра съезжали на 16 мм.
    boundary = std::min(boundary, shortestEdge(reference, loop) / 100);
    for (size_t k = 0; k < loop.size() && polygon.size() > 0; ++k) {
        const Vector3 tail = reference.vertices[size_t(loop[k])], head = reference.vertices[size_t(loop[(k + 1) % loop.size()])];
        // Обход грани идёт против часовой стрелки, поэтому наружу от ребра смотрит edge × n.
        polygon = clip(polygon, vertexOf, (head - tail).cross(n), tail, boundary, int(k), scratch);
        std::swap(vertexOf, scratch);
    }
    ContactCandidates near;
    for (ContactCandidate p : polygon) {
        p.separation = n.dot(p.point - reference.pointOnFace(face));
        if (p.separation <= margin) near.push_back(p);
    }
    addNearVertices(reference, face, incident, incidentLoop, margin, boundary, near);
    return ContactCandidate::ordered(near, touching);
}

// Ближайшие точки двух скрещённых рёбер: на прямых p + s d, отрезками они ограничены здесь же.
void closestOnEdges(Vector3 p1, Vector3 d1, Vector3 p2, Vector3 d2, Vector3& on1, Vector3& on2) {
    const Vector3 r = p1 - p2;
    const Real a = d1.dot(d1), e = d2.dot(d2), b = d1.dot(d2), c = d1.dot(r), f = d2.dot(r);
    const Real denominator = a * e - b * b; // не ноль: параллельные рёбра сюда не попадают (Sat.cpp)
    const Real s = std::min(Real(1), std::max(Real(0), (b * f - c * e) / denominator));
    const Real t = std::min(Real(1), std::max(Real(0), (b * s + f) / e));
    on1 = p1 + s * d1;
    on2 = p2 + t * d2;
}

// Окончательная ось контакта; false — тела дальше запаса.
bool settleAxis(SeparatingAxis& axis, const TransformedHull& A, const TransformedHull& B, Real margin, Real touching, Real boundary) {
    // С телом, у которого наружу обращена одна грань, контакт идёт только через неё.
    if (SeparatingAxis outer; outerFaceAxis(A, B, outer)) {
        if (outer.separation > margin) return false;
        axis = outer;
    }
    // Ось пары рёбер даёт одну точку. Если ось грани хуже неё меньше чем на полосу касания, грани
    // почти параллельны и касаются площадкой: все её точки уже в полосе, и удар обязан видеть их
    // все. С одной точкой он останавливал сближение только в ней, а остальная грань шла насквозь:
    // два бруска на 100 м/с входили друг в друга на 36 см. Зазор каждой точки площадки меряется
    // от плоскости грани в самой точке, поэтому точность касания от замены оси не страдает.
    if (axis.kind == SeparatingAxis::Kind::EdgePair) {
        const SeparatingAxis face = findFaceAxis(A, B, boundary);
        const bool onB = face.kind == SeparatingAxis::Kind::FaceB;
        if (axis.separation - face.separation <= touching &&
            clipAgainstFace(onB ? B : A, onB ? face.featureB : face.featureA, onB ? A : B, margin, touching, boundary).size() > 0)
            axis = face;
    }
    return true;
}

} // namespace

ContactManifold HullVsHullAlgorithm::clipFaces(const ConvexHull& a, const Transform& poseA, const ConvexHull& b, const Transform& poseB, Real margin, Real touching, Real boundary) {
    ContactManifold manifold;
    // Как и разделяющая ось, работа идёт в осях тела A; в мир переводится ответ.
    const Transform toB = poseA.relative(poseB);
    const TransformedHull A(a, Transform{}), B(b, toB);
    SeparatingAxis axis = findSeparatingAxis(A, B, boundary);
    if (axis.separation > margin) return manifold;
    if (!settleAxis(axis, A, B, margin, touching, boundary)) return manifold;
    manifold.normal = poseA.direction(axis.normal);

    const auto add = [&](Vector3 onA, Vector3 onB, Real separation, std::uint32_t id) {
        manifold.add({poseA.point(onA), poseA.point(onB), separation, id});
    };
    if (axis.kind == SeparatingAxis::Kind::EdgePair) {
        const ConvexHull::Edge& ea = a.edges()[size_t(axis.featureA)];
        const ConvexHull::Edge& eb = b.edges()[size_t(axis.featureB)];
        Vector3 onA, onB;
        closestOnEdges(A.vertices[size_t(ea.tail)], A.vertices[size_t(ea.head)] - A.vertices[size_t(ea.tail)],
                       B.vertices[size_t(eb.tail)], B.vertices[size_t(eb.head)] - B.vertices[size_t(eb.tail)], onA, onB);
        add(onA, onB, axis.separation, 0xC0000000u | std::uint32_t(axis.featureA) << 12 | std::uint32_t(axis.featureB));
    } else if (axis.kind == SeparatingAxis::Kind::FaceA) {
        // Точки лежат на грани B; на A — их проекции на плоскость опорной грани.
        for (const ContactCandidate& p : clipAgainstFace(A, axis.featureA, B, margin, touching, boundary))
            add(p.point - p.separation * axis.normal, p.point, p.separation, p.id);
    } else {
        // Опорная грань у B: точки лежат на грани A, нормаль от A к B противоположна нормали грани.
        manifold.faceOnB = true;
        for (const ContactCandidate& p : clipAgainstFace(B, axis.featureB, A, margin, touching, boundary))
            add(p.point, p.point + p.separation * axis.normal, p.separation, 0x40000000u | p.id);
    }
    return manifold;
}

ContactManifold HullVsHullAlgorithm::collide(const ConvexShape& a, const Transform& poseA, const ConvexShape& b, const Transform& poseB, const ClosestPoints& closest, Real margin,
                                             Real touching, Real boundary) const {
    const ConvexHull &hullA = *a.hull(), &hullB = *b.hull();
    ContactManifold manifold = clipFaces(hullA, poseA, hullB, poseB, margin, touching, boundary);
    // Отсечение граней даёт точки, только когда грани перекрываются в плане. Если тела
    // касаются вершиной или ребром в стороне от грани, отсечение не оставляет ни одной точки,
    // хотя касание есть; тогда контакт — ближайшие точки, как у скруглённых форм.
    if (manifold.count > 0 || closest.coresOverlap) return manifold;
    // У призмы рельефа запасного контакта «ближайшими точками» нет: рядом со швом ближайшие
    // точки — ребро бруска и боковое ребро призмы, нормаль между ними почти горизонтальна,
    // и такой контакт останавливал бы брусок, подъезжающий к шву.
    if (hullA.outerFace >= 0 || hullB.outerFace >= 0) return manifold;
    if (closest.distance > margin) return manifold;
    manifold.normal = closest.normal;
    manifold.roundA = a.radius, manifold.roundB = b.radius;
    manifold.add({closest.onA, closest.onB, closest.distance, 0});
    return manifold;
}

} // namespace kp
