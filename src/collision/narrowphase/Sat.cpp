#include <kp/collision/narrowphase/Sat.h>

#include <kp/collision/narrowphase/TransformedHull.h>

namespace kp {
namespace {

// Лучшая ось среди нормалей граней тела `of`: зазор вдоль нормали грани равен расстоянию от
// плоскости грани до самой глубокой вершины другого тела.
void faceAxis(const TransformedHull& of, const TransformedHull& other, Real& separation, int& face) {
    separation = -std::numeric_limits<Real>::infinity();
    for (int i = 0; i < int(of.normals.size()); ++i) {
        const Vector3 n = of.normals[size_t(i)];
        const Real s = n.dot(other.support(-n) - of.pointOnFace(i));
        if (s > separation) separation = s, face = i;
    }
}

// Образуют ли два ребра грань разности Минковского. На единичной сфере ребру A соответствует
// дуга между нормалями его граней a и b, ребру B — дуга между −c и −d; пара даёт грань разности,
// если дуги пересекаются. Сюда c и d передаются уже со знаком минус.
//
// Конец одной дуги, лежащий на большом круге другой, — не пересечение: знак нулевого смешанного
// произведения решает округление, и проверка полушария за ним ошибается. Так вертикальное ребро
// коробки «образовывало грань» с горизонтальным ребром дальней стороны тонкой пластины, и зазор
// выходил больше истинного на её толщину. Ось такой пары совпадает с нормалью грани и ею покрыта.
bool formsMinkowskiFace(Vector3 a, Vector3 b, Vector3 c, Vector3 d) {
    const Vector3 bxa = b.cross(a), dxc = d.cross(c);
    const Real cba = c.dot(bxa), dba = d.dot(bxa), adc = a.dot(dxc), bdc = b.dot(dxc);
    const Real onArc = Real(1e-10);
    for (Real side : {cba, dba, adc, bdc})
        if (abs(side) < onArc) return false;
    return cba * dba < 0 && adc * bdc < 0 && cba * bdc > 0;
}

} // namespace

// Работа идёт в осях тела A (см. Transform::relative); в мир переводится только нормаль.
static SeparatingAxis leastPenetrationInA(const ConvexHull& a, const ConvexHull& b, const Transform& toB, Real preference);

SeparatingAxis findSeparatingAxis(const ConvexHull& a, const Transform& poseA, const ConvexHull& b, const Transform& poseB, Real preference) {
    SeparatingAxis axis = leastPenetrationInA(a, b, poseA.relative(poseB), preference);
    axis.normal = poseA.direction(axis.normal);
    return axis;
}

static SeparatingAxis bestFaceInA(const TransformedHull& A, const TransformedHull& B, Real preference) {
    Real separationA, separationB;
    int faceA = 0, faceB = 0;
    faceAxis(A, B, separationA, faceA);
    faceAxis(B, A, separationB, faceB);
    if (separationB > separationA + preference) return {SeparatingAxis::Kind::FaceB, separationB, -B.normals[size_t(faceB)], 0, faceB};
    return {SeparatingAxis::Kind::FaceA, separationA, A.normals[size_t(faceA)], faceA, 0};
}

bool outerFaceAxis(const TransformedHull& A, const TransformedHull& B, SeparatingAxis& axis) {
    if (A.shape.outerFace >= 0) {
        const Vector3 n = A.normals[size_t(A.shape.outerFace)];
        axis = {SeparatingAxis::Kind::FaceA, n.dot(B.support(-1 * n) - A.pointOnFace(A.shape.outerFace)), n, A.shape.outerFace, 0};
        return true;
    }
    if (B.shape.outerFace >= 0) {
        const Vector3 n = B.normals[size_t(B.shape.outerFace)];
        axis = {SeparatingAxis::Kind::FaceB, n.dot(A.support(-1 * n) - B.pointOnFace(B.shape.outerFace)), -1 * n, 0, B.shape.outerFace};
        return true;
    }
    return false;
}

SeparatingAxis findFaceAxis(const ConvexHull& a, const Transform& poseA, const ConvexHull& b, const Transform& poseB, Real preference) {
    SeparatingAxis axis = bestFaceInA(TransformedHull(a, Transform{}), TransformedHull(b, poseA.relative(poseB)), preference);
    axis.normal = poseA.direction(axis.normal);
    return axis;
}

static SeparatingAxis leastPenetrationInA(const ConvexHull& a, const ConvexHull& b, const Transform& toB, Real preference) {
    const TransformedHull A(a, Transform{}), B(b, toB);
    SeparatingAxis best = bestFaceInA(A, B, preference);

    for (int i = 0; i < int(a.edges().size()); ++i) {
        const ConvexHull::Edge& ea = a.edges()[size_t(i)];
        const Vector3 tailA = A.vertices[size_t(ea.tail)], directionA = A.vertices[size_t(ea.head)] - tailA;
        for (int j = 0; j < int(b.edges().size()); ++j) {
            const ConvexHull::Edge& eb = b.edges()[size_t(j)];
            if (!formsMinkowskiFace(A.normals[size_t(ea.left)], A.normals[size_t(ea.right)],
                                    -B.normals[size_t(eb.left)], -B.normals[size_t(eb.right)]))
                continue;
            const Vector3 tailB = B.vertices[size_t(eb.tail)], directionB = B.vertices[size_t(eb.head)] - tailB;
            const Vector3 axis = directionA.cross(directionB);
            const Real axisLength = axis.length();
            // Почти параллельные рёбра оси не задают; такой случай покрыт осями граней.
            if (axisLength <= 64 * kEpsilon * directionA.length() * directionB.length()) continue;
            // Ось направляется наружу тела A — от его центра к ребру.
            const Vector3 normal = (axis.dot(tailA - A.centre) < 0 ? -1 / axisLength : 1 / axisLength) * axis;
            const Real s = normal.dot(tailB - tailA);
            if (s > best.separation + (best.kind == SeparatingAxis::Kind::EdgePair ? 0 : preference))
                best = {SeparatingAxis::Kind::EdgePair, s, normal, i, j};
        }
    }
    return best;
}

} // namespace kp
