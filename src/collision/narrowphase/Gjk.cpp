#include <kp/collision/shapes/Cylinder.h>
#include <kp/collision/narrowphase/Gjk.h>

#include <optional>

#include <algorithm>

namespace kp {
namespace {

// Вершина симплекса: точка разности w = a − b и породившие её точки тел.
struct Vertex {
    Vector3 w, a, b;
};

struct Simplex {
    Vertex vertex[4];
    Real weight[4] = {0, 0, 0, 0}; // барицентрические веса ближайшей к началу точки
    int count = 0;

    Vector3 closest() const {
        Vector3 v;
        for (int i = 0; i < count; ++i) v += weight[i] * vertex[i].w;
        return v;
    }
    // Оставляет только вершины с ненулевым весом: они несут ближайшую точку.
    void reduce() {
        int kept = 0;
        for (int i = 0; i < count; ++i)
            if (weight[i] > 0) vertex[kept] = vertex[i], weight[kept] = weight[i], ++kept;
        count = kept;
    }
};

// Ближайшая к началу координат точка отрезка ab: веса вершин.
void closestOnSegment(Vector3 a, Vector3 b, Real& wa, Real& wb) {
    const Vector3 ab = b - a;
    const Real t = -a.dot(ab) / ab.dot(ab);
    wb = t <= 0 ? 0 : t >= 1 ? 1 : t;
    wa = 1 - wb;
}

// Ближайшая к началу координат точка треугольника abc: веса вершин.
// Разбор областей Вороного вершин, рёбер и грани: Ericson, «Real-Time Collision Detection»,
// раздел 5.1.5.
void closestOnTriangle(Vector3 a, Vector3 b, Vector3 c, Real& wa, Real& wb, Real& wc) {
    const Vector3 ab = b - a, ac = c - a;
    const Real d1 = -ab.dot(a), d2 = -ac.dot(a);
    if (d1 <= 0 && d2 <= 0) return void((wa = 1, wb = 0, wc = 0)); // область вершины a
    const Real d3 = -ab.dot(b), d4 = -ac.dot(b);
    if (d3 >= 0 && d4 <= d3) return void((wa = 0, wb = 1, wc = 0)); // вершина b
    const Real vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) { // ребро ab
        wb = d1 / (d1 - d3);
        return void((wa = 1 - wb, wc = 0));
    }
    const Real d5 = -ab.dot(c), d6 = -ac.dot(c);
    if (d6 >= 0 && d5 <= d6) return void((wa = 0, wb = 0, wc = 1)); // вершина c
    const Real vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) { // ребро ac
        wc = d2 / (d2 - d6);
        return void((wa = 1 - wc, wb = 0));
    }
    const Real va = d3 * d6 - d5 * d4;
    if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) { // ребро bc
        wc = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return void((wa = 0, wb = 1 - wc));
    }
    const Real sum = va + vb + vc; // внутренность грани
    wb = vb / sum, wc = vc / sum, wa = 1 - wb - wc;
}

// Обновляет веса симплекса так, чтобы они задавали ближайшую к началу координат точку.
// Возвращает false, если начало координат лежит внутри тетраэдра: формы перекрылись.
bool solve(Simplex& s) {
    const Vector3 a = s.vertex[0].w, b = s.vertex[1].w, c = s.vertex[2].w, d = s.vertex[3].w;
    Real* w = s.weight;
    if (s.count == 1) return w[0] = 1, true;
    if (s.count == 2) return closestOnSegment(a, b, w[0], w[1]), true;
    if (s.count == 3) return closestOnTriangle(a, b, c, w[0], w[1], w[2]), true;

    // Тетраэдр: начало координат снаружи той грани, от которой оно лежит по другую сторону, чем
    // противоположная вершина. Среди таких граней берётся ближайшая точка (Ericson, раздел 5.1.6).
    //
    // Когда тела обращены друг к другу почти параллельными гранями, четыре опорные точки лежат
    // почти в одной плоскости и тетраэдр плоский. У плоского тетраэдра нет внутренности, а знак
    // «по какую сторону противоположная вершина» решается округлением; такая грань проверяется
    // всегда, иначе шум объявляет перекрытием зазор в сантиметр.
    const int faces[4][4] = {{0, 1, 2, 3}, {0, 3, 1, 2}, {0, 2, 3, 1}, {1, 3, 2, 0}};
    const Vector3 p[4] = {a, b, c, d};
    Real extent = 0;
    for (const Vector3& vertex : p) extent = std::max(extent, vertex.length());
    Real bestDistance = -1, best[4] = {0, 0, 0, 0};
    for (const int* f : faces) {
        const Vector3 normal = (p[f[1]] - p[f[0]]).cross(p[f[2]] - p[f[0]]);
        const Real origin = -p[f[0]].dot(normal), opposite = (p[f[3]] - p[f[0]]).dot(normal);
        const bool flat = abs(opposite) <= 64 * kEpsilon * normal.length() * extent;
        if (!flat && origin * opposite > 0) continue; // начало с той же стороны, что и противоположная вершина
        Real candidate[4] = {0, 0, 0, 0};
        closestOnTriangle(p[f[0]], p[f[1]], p[f[2]], candidate[f[0]], candidate[f[1]], candidate[f[2]]);
        Vector3 point;
        for (int i = 0; i < 4; ++i) point += candidate[i] * p[i];
        const Real distance = point.dot(point);
        if (bestDistance < 0 || distance < bestDistance) {
            bestDistance = distance;
            for (int i = 0; i < 4; ++i) best[i] = candidate[i];
        }
    }
    if (bestDistance < 0) return false;
    for (int i = 0; i < 4; ++i) w[i] = best[i];
    return true;
}

} // namespace

// Расстояние между ядрами как они есть.
static ClosestPoints betweenCores(const ConvexShape& a, const Transform& poseA, const ConvexShape& b, const Transform& poseB) {
    // Вся работа идёт в осях тела A: там числа порядка размеров тел, а не их удалённости от
    // начала координат. В мир переводится только ответ.
    const Transform toB = poseA.relative(poseB);

    // Опорная точка разности в направлении d: самая далёкая точка A вдоль d минус самая далёкая
    // точка B вдоль −d.
    const auto support = [&](Vector3 d) {
        const Vector3 onA = a.support(d);
        const Vector3 onB = toB.point(b.support(toB.localDirection(-d)));
        return Vertex{onA - onB, onA, onB};
    };

    Simplex simplex;
    simplex.vertex[0] = support(toB.position);
    simplex.weight[0] = 1;
    simplex.count = 1;
    Vector3 v = simplex.vertex[0].w;

    ClosestPoints result;
    // Квадрат длины, ниже которого v считается нулём: относительно размера самой разности.
    Real scale = v.dot(v);
    for (result.iterations = 1; result.iterations <= 64; ++result.iterations) {
        if (v.dot(v) <= kEpsilon * kEpsilon * scale) return result.coresOverlap = true, result;
        const Vertex next = support(-v);
        scale = std::max(scale, next.w.dot(next.w));
        // v·v − v·w — насколько новая опорная точка ближе к началу вдоль v. Если продвижения
        // нет, v уже ближайшая точка разности.
        if (v.dot(v) - v.dot(next.w) <= 64 * kEpsilon * v.dot(v)) break;
        // Опорная точка, которая уже есть в симплексе, ничего не добавит: дальше только цикл.
        bool repeated = false;
        for (int i = 0; i < simplex.count; ++i)
            repeated = repeated || (next.w.x == simplex.vertex[i].w.x && next.w.y == simplex.vertex[i].w.y &&
                                    next.w.z == simplex.vertex[i].w.z);
        if (repeated) break;
        const Simplex previous = simplex;
        simplex.vertex[simplex.count++] = next;
        if (!solve(simplex)) return result.coresOverlap = true, result;
        simplex.reduce();
        const Vector3 closer = simplex.closest();
        // Расстояние обязано убывать. Если из-за округления оно не убыло, лучший ответ — прежний.
        if (closer.dot(closer) >= v.dot(v)) {
            simplex = previous;
            break;
        }
        v = closer;
    }

    const Real coreDistance = v.length();
    if (coreDistance == 0) return result.coresOverlap = true, result;
    const Vector3 normal = (-1 / coreDistance) * v; // v направлен от B к A
    Vector3 onA, onB;
    for (int i = 0; i < simplex.count; ++i) {
        onA += simplex.weight[i] * simplex.vertex[i].a;
        onB += simplex.weight[i] * simplex.vertex[i].b;
    }
    result.normal = poseA.direction(normal);
    result.onA = poseA.point(onA + a.radius * normal);
    result.onB = poseA.point(onB + -b.radius * normal);
    result.distance = coreDistance - a.radius - b.radius;
    return result;
}

// У цилиндра ядро — он сам или почти он сам, поэтому его ядро перекрывается с другим телом при
// первом же касании, и глубина пропадает именно тогда, когда она нужна: острый цилиндр, лежащий
// на грани, «перекрыт» всегда. Глубину возвращает то же ядро, ужатое на shrink: ужатый цилиндр со
// скруглением на shrink больше — та же форма, кроме кромки, которая скруглена сильнее. Пока
// перекрытие мельче shrink, ужатые ядра разделены и расстояние между ними известно; отличие от
// настоящей формы — не больше 0,41 shrink и только у кромки. Если мало — ужатие растёт.
constexpr Real kCoreShrink = Real(1e-6); // м: первое ужатие, допуск перекрытия

ClosestPoints closestPoints(const ConvexShape& a, const Transform& poseA, const ConvexShape& b, const Transform& poseB) {
    ClosestPoints result = betweenCores(a, poseA, b, poseB);
    const auto* cylinderA = dynamic_cast<const Cylinder*>(&a);
    const auto* cylinderB = dynamic_cast<const Cylinder*>(&b);
    if (!result.coresOverlap || (!cylinderA && !cylinderB)) return result;
    // Ужать ядро можно не больше чем до девяти десятых его меньшего размера.
    const auto limit = [](const Cylinder* c) { return c ? Real(0.9) * std::min(c->coreHalfHeight, c->coreRadius) : Real(0); };
    const auto shrunk = [](const Cylinder& c, Real by) { return Cylinder(c.coreHalfHeight + c.radius, c.coreRadius + c.radius, c.radius + by); };
    for (Real shrink = kCoreShrink;; shrink *= 8) {
        const Real byA = std::min(shrink, limit(cylinderA)), byB = std::min(shrink, limit(cylinderB));
        const std::optional<Cylinder> smallerA = cylinderA ? std::optional<Cylinder>(shrunk(*cylinderA, byA)) : std::nullopt;
        const std::optional<Cylinder> smallerB = cylinderB ? std::optional<Cylinder>(shrunk(*cylinderB, byB)) : std::nullopt;
        const int spent = result.iterations;
        result = betweenCores(smallerA ? static_cast<const ConvexShape&>(*smallerA) : a, poseA, smallerB ? static_cast<const ConvexShape&>(*smallerB) : b, poseB);
        result.iterations += spent;
        if (!result.coresOverlap || (shrink >= limit(cylinderA) && shrink >= limit(cylinderB))) return result;
    }
}

} // namespace kp
