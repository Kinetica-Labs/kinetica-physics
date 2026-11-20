#include <collision/narrowphase/Sat.h>

#include <collision/narrowphase/TransformedHull.h>
#include <math/StepMemory.h>

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
// если дуги пересекаются. Признак — знаки четырёх смешанных произведений: c · (b × a), d · (b × a),
// a · (d × c), b · (d × c), где c и d уже со знаком минус (arcSides).
//
// Конец одной дуги, лежащий на большом круге другой, — не пересечение: знак нулевого смешанного
// произведения решает округление, и проверка полушария за ним ошибается. Так вертикальное ребро
// коробки «образовывало грань» с горизонтальным ребром дальней стороны тонкой пластины, и зазор
// выходил больше истинного на её толщину. Ось такой пары совпадает с нормалью грани и ею покрыта.
bool formsMinkowskiFace(Real cba, Real dba, Real adc, Real bdc) {
    const Real onArc = Real(1e-10);
    for (Real side : {cba, dba, adc, bdc})
        if (abs(side) < onArc) return false;
    return cba * dba < 0 && adc * bdc < 0 && cba * bdc > 0;
}

// Смешанные произведения для проверки грани разности: строка — ребро тела `of`, столбец — грань
// тела `other`, значение — (sign · нормаль грани) · (правая нормаль ребра × левая). Произведение
// зависит от ребра одного тела и грани другого, а не от пары рёбер: у двух кубов это 2 · 12 · 6
// скалярных произведений вместо 4 на каждую из 144 пар рёбер.
StepArray<Real> arcSides(const TransformedHull& of, const TransformedHull& other, Real sign) {
    StepArray<Real> sides;
    sides.reserve(of.shape.edges().size() * other.normals.size());
    for (const ConvexHull::Edge& edge : of.shape.edges()) {
        const Vector3 arc = of.normals[size_t(edge.right)].cross(of.normals[size_t(edge.left)]);
        for (Vector3 normal : other.normals) sides.push_back((sign * normal).dot(arc));
    }
    return sides;
}

} // namespace

// Оси пар рёбер: лучше оси best — заменяют её.
static SeparatingAxis bestEdgePair(const TransformedHull& A, const TransformedHull& B, SeparatingAxis best, Real preference) {
    const size_t facesA = A.normals.size(), facesB = B.normals.size();
    const StepArray<Real> sidesA = arcSides(A, B, -1), sidesB = arcSides(B, A, 1);
    for (int i = 0; i < int(A.shape.edges().size()); ++i) {
        const ConvexHull::Edge& ea = A.shape.edges()[size_t(i)];
        const Vector3 tailA = A.vertices[size_t(ea.tail)], directionA = A.vertices[size_t(ea.head)] - tailA;
        const Real* ofA = &sidesA[size_t(i) * facesB];
        for (int j = 0; j < int(B.shape.edges().size()); ++j) {
            const ConvexHull::Edge& eb = B.shape.edges()[size_t(j)];
            const Real* ofB = &sidesB[size_t(j) * facesA];
            if (!formsMinkowskiFace(ofA[eb.left], ofA[eb.right], ofB[ea.left], ofB[ea.right])) continue;
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

static SeparatingAxis bestFaceInA(const TransformedHull& A, const TransformedHull& B, Real preference) {
    Real separationA, separationB;
    int faceA = 0, faceB = 0;
    faceAxis(A, B, separationA, faceA);
    faceAxis(B, A, separationB, faceB);
    if (separationB > separationA + preference) return {SeparatingAxis::Kind::FaceB, separationB, -B.normals[size_t(faceB)], 0, faceB};
    return {SeparatingAxis::Kind::FaceA, separationA, A.normals[size_t(faceA)], faceA, 0};
}

// Оболочки уже в общих осях: ось — в них же. Так отсечение граней строит оболочки один раз на пару.
SeparatingAxis findSeparatingAxis(const TransformedHull& A, const TransformedHull& B, Real preference) { return bestEdgePair(A, B, bestFaceInA(A, B, preference), preference); }
SeparatingAxis findFaceAxis(const TransformedHull& A, const TransformedHull& B, Real preference) { return bestFaceInA(A, B, preference); }

// Работа идёт в осях тела A (см. Transform::relative); в мир переводится только нормаль.
SeparatingAxis findSeparatingAxis(const ConvexHull& a, const Transform& poseA, const ConvexHull& b, const Transform& poseB, Real preference) {
    SeparatingAxis axis = findSeparatingAxis(TransformedHull(a, Transform{}), TransformedHull(b, poseA.relative(poseB)), preference);
    axis.normal = poseA.direction(axis.normal);
    return axis;
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

} // namespace kp
