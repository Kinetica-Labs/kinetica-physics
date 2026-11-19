// Геометрия против формул: расстояния и глубины, у которых есть точный ответ.

#include <body/CompoundBodyBuilder.h>
#include <collision/narrowphase/HullVsHullAlgorithm.h>
#include "Check.h"

#include <collision/shapes/Capsule.h>
#include <collision/shapes/Sphere.h>
#include <body/Body.h>
#include <body/MassFrame.h>
#include <collision/narrowphase/Gjk.h>
#include <collision/narrowphase/ContactManifold.h>
#include <collision/shapes/Meshes.h>
#include <collision/narrowphase/Sat.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

using namespace kp;

namespace {

// Тесты строят повороты обычной тригонометрией: правило «без libm» касается ядра, а не проверок.
Quaternion aboutAxis(Vector3 axis, double degrees) {
    const double half = degrees * 3.14159265358979323846 / 360;
    return {std::cos(half), std::sin(half) * axis.x, std::sin(half) * axis.y, std::sin(half) * axis.z};
}
Transform at(Vector3 position, Quaternion orientation = {}) { return {position, orientation}; }

const double kExact = 1e-12; // допуск на величины, у которых есть формула

void polytopeTopology() {
    const ConvexHull box = ConvexHull::box({1, 2, 3});
    CHECK(box.vertices().size() == 8 && box.faces().size() == 6 && box.edges().size() == 12, "V=%zu F=%zu E=%zu",
          box.vertices().size(), box.faces().size(), box.edges().size());
    // Нормали единичные, наружу, плоскости на расстоянии полуразмеров: −x, +x, −y, +y, −z, +z.
    const Vector3 normals[6] = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};
    const double offsets[6] = {1, 1, 2, 2, 3, 3};
    for (int i = 0; i < 6; ++i) {
        CHECK_NEAR((box.faces()[size_t(i)].normal - normals[i]).length(), 0, kExact);
        CHECK_NEAR(box.faces()[size_t(i)].offset, offsets[i], kExact);
    }
    for (const ConvexHull::Edge& e : box.edges()) CHECK(e.left != e.right, "ребро %d-%d лежит в одной грани", e.tail, e.head);
}

void spheres() {
    const Sphere a(0.5), b(1.0);
    const ClosestPoints apart = closestPoints(a, at({0, 0, 0}), b, at({3, 0, 0}));
    CHECK_NEAR(apart.distance, 1.5, kExact);
    CHECK_NEAR((apart.normal - Vector3{1, 0, 0}).length(), 0, kExact);
    CHECK_NEAR((apart.onA - Vector3{0.5, 0, 0}).length(), 0, kExact);
    CHECK_NEAR((apart.onB - Vector3{2, 0, 0}).length(), 0, kExact);
    // Оболочки перекрылись, ядра разделены: глубина известна точно.
    const ClosestPoints overlapping = closestPoints(a, at({0, 0, 0}), b, at({0, 1, 0}));
    CHECK_NEAR(overlapping.distance, -0.5, kExact);
    CHECK(!overlapping.coresOverlap, "ядра двух разных точек не перекрываются");
}

void boxes() {
    const ConvexHull big = ConvexHull::box({1, 1, 1}), small = ConvexHull::box({0.5, 0.5, 0.5});
    CHECK_NEAR(closestPoints(big, at({}), small, at({3, 0, 0})).distance, 1.5, kExact);                   // грань к грани
    CHECK_NEAR(closestPoints(big, at({}), small, at({2, 2, 0})).distance, 0.5 * std::sqrt(2.0), kExact);  // ребро к ребру
    CHECK_NEAR(closestPoints(big, at({}), small, at({2, 2, 2})).distance, 0.5 * std::sqrt(3.0), kExact);  // вершина к вершине
    // Куб, повёрнутый на 45° вокруг Z, обращён к соседу ребром: оно на расстоянии √2 от центра.
    const ClosestPoints edgeOn = closestPoints(big, at({}), big, at({3, 0, 0}, aboutAxis({0, 0, 1}, 45)));
    CHECK_NEAR(edgeOn.distance, 3 - std::sqrt(2.0) - 1, kExact);
    CHECK_NEAR((edgeOn.normal - Vector3{1, 0, 0}).length(), 0, 1e-9);
    CHECK(closestPoints(big, at({}), big, at({1.5, 0.3, 0.2})).coresOverlap, "пересекающиеся кубы должны дать перекрытие");
}

void roundedShapes() {
    const ConvexHull box = ConvexHull::box({1, 1, 1});
    const Capsule capsule(1.0, 0.25);
    const Sphere ball(0.5);
    CHECK_NEAR(closestPoints(box, at({}), capsule, at({2, 0, 0})).distance, 0.75, kExact);
    const ClosestPoints onTop = closestPoints(box, at({}), ball, at({0.2, 3, -0.3}));
    CHECK_NEAR(onTop.distance, 1.5, kExact);
    CHECK_NEAR((onTop.onA - Vector3{0.2, 1, -0.3}).length(), 0, 1e-9);
    // Капсула, лежащая поперёк: отрезок вдоль X после поворота на 90° вокруг Z.
    CHECK_NEAR(closestPoints(capsule, at({0, 0, 0}, aboutAxis({0, 0, 1}, 90)), ball, at({3, 0, 0})).distance, 3 - 1 - 0.25 - 0.5, kExact);
}

void separatingAxes() {
    const ConvexHull cube = ConvexHull::box({1, 1, 1});
    const SeparatingAxis overlap = findSeparatingAxis(cube, at({}), cube, at({1.9, 0, 0}));
    CHECK_NEAR(overlap.separation, -0.1, kExact);
    CHECK(overlap.kind == SeparatingAxis::Kind::FaceA && overlap.featureA == 1, "ожидалась грань +x тела A");
    CHECK_NEAR((overlap.normal - Vector3{1, 0, 0}).length(), 0, kExact);
    CHECK_NEAR(findSeparatingAxis(cube, at({}), cube, at({2.5, 0, 0})).separation, 0.5, kExact);

    // Скрещённые рёбра: A повёрнут на 45° вокруг Z (ребро вдоль Z на x = √2), B — на 45° вокруг Y
    // (ребро вдоль Y на x = c − √2). Наименьшее перекрытие — вдоль X, и даёт его пара рёбер.
    const double c = 2 * std::sqrt(2.0) - 0.1;
    const SeparatingAxis crossed = findSeparatingAxis(cube, at({}, aboutAxis({0, 0, 1}, 45)), cube, at({c, 0, 0}, aboutAxis({0, 1, 0}, 45)));
    CHECK(crossed.kind == SeparatingAxis::Kind::EdgePair, "ожидалась пара рёбер");
    CHECK_NEAR(crossed.separation, -0.1, 1e-9);
    CHECK_NEAR((crossed.normal - Vector3{1, 0, 0}).length(), 0, 1e-9);
}

// Детерминированный датчик: линейный конгруэнтный, числа в [0, 1).
struct Sequence {
    unsigned long long state = 88172645463325252ull;
    double next() {
        state = state * 6364136223846793005ull + 1442695040888963407ull;
        return double(state >> 11) / 9007199254740992.0;
    }
    double between(double lo, double hi) { return lo + (hi - lo) * next(); }
};

// Две независимые теоремы обязаны согласиться на любых позах: GJK говорит «ядра перекрылись»
// тогда и только тогда, когда разделяющей оси нет; а зазор вдоль любой оси не больше расстояния.
void gjkAgreesWithSat() {
    Sequence random;
    int separated = 0, overlapping = 0, worstIterations = 0;
    for (int trial = 0; trial < 5000; ++trial) {
        const ConvexHull a = ConvexHull::box({random.between(0.2, 1.5), random.between(0.2, 1.5), random.between(0.2, 1.5)});
        const ConvexHull b = ConvexHull::box({random.between(0.2, 1.5), random.between(0.2, 1.5), random.between(0.2, 1.5)});
        const auto pose = [&] {
            const Quaternion q = Quaternion{random.between(-1, 1), random.between(-1, 1), random.between(-1, 1), random.between(-1, 1)}.unit();
            return Transform{{random.between(-2, 2), random.between(-2, 2), random.between(-2, 2)}, q};
        };
        const Transform pa = pose(), pb = pose();
        const ClosestPoints gjk = closestPoints(a, pa, b, pb);
        const SeparatingAxis sat = findSeparatingAxis(a, pa, b, pb);
        worstIterations = worstIterations > gjk.iterations ? worstIterations : gjk.iterations;
        if (gjk.coresOverlap) {
            ++overlapping;
            CHECK(sat.separation <= 1e-9, "проба %d: GJK — перекрытие, а SAT нашёл зазор %.3g", trial, sat.separation);
        } else {
            ++separated;
            CHECK(sat.separation > -1e-9, "проба %d: GJK — расстояние %.3g, а SAT — перекрытие %.3g", trial, gjk.distance, sat.separation);
            CHECK(sat.separation <= gjk.distance + 1e-9, "проба %d: зазор по оси %.12g больше расстояния %.12g", trial, sat.separation, gjk.distance);
            CHECK_NEAR((gjk.onB - gjk.onA).length(), gjk.distance, 1e-9);
            CHECK_NEAR(gjk.normal.dot(gjk.onB - gjk.onA), gjk.distance, 1e-9);
        }
    }
    std::printf("  GJK против SAT: 5000 пар брусков, разделены %d, перекрылись %d, наибольшее число итераций GJK %d\n",
                separated, overlapping, worstIterations);
}

// Почти параллельные грани: четыре опорные точки GJK лежат почти в одной плоскости, и симплекс
// вырождается. Ошибка, найденная анализом tests/ErrorAnalysis.cpp: зазор до сантиметра
// объявлялся перекрытием. Разделяющая ось здесь точна, и GJK обязан с ней согласиться.
void nearlyParallelFaces() {
    Sequence random;
    int falseOverlaps = 0;
    for (int trial = 0; trial < 5000; ++trial) {
        const ConvexHull a = ConvexHull::box({random.between(0.1, 2), random.between(0.1, 2), random.between(0.1, 2)});
        const ConvexHull b = ConvexHull::box({random.between(0.1, 2), random.between(0.1, 2), random.between(0.1, 2)});
        const Vector3 tilt = std::pow(10.0, random.between(-12, -1)) * Vector3{random.between(-1, 1), random.between(-1, 1), random.between(-1, 1)};
        const double gap = std::pow(10.0, random.between(-6, -1));
        const double reach = a.vertices()[7].x + b.vertices()[7].x; // сумма полуразмеров по X
        const Transform pb{{reach + gap, random.between(-0.05, 0.05), 0}, Quaternion::cayley(tilt)};
        const SeparatingAxis sat = findSeparatingAxis(a, at({}), b, pb);
        falseOverlaps += closestPoints(a, at({}), b, pb).coresOverlap && sat.separation > 1e-9;
    }
    CHECK(falseOverlaps == 0, "GJK объявил перекрытие при зазоре в %d пробах из 5000", falseOverlaps);
}

// Коробки, повёрнутые вокруг общей оси: нормали их граней либо совпадают, либо перпендикулярны, и
// дуги рёбер на сфере Гаусса касаются концами. Ошибка, найденная проверкой столкновений от одного
// вращения: такая пара рёбер принималась за грань разности Минковского, и тонкой пластине,
// уже вошедшей в коробку, приписывался зазор в её толщину. Зазор вдоль любой оси не больше
// расстояния между телами — GJK обязан его подтвердить.
void boxesAboutCommonAxis() {
    Sequence random;
    int falseGaps = 0;
    const ConvexHull plate = ConvexHull::box({0.3, 0.2, 0.01}), wall = ConvexHull::box({0.01, 0.002, 0.3});
    for (int trial = 0; trial < 5000; ++trial) {
        // Наклон на уровне округления — такой оставляет после себя интегрирование поворота.
        const Vector3 tilt = std::pow(10.0, random.between(-17, -13)) * Vector3{random.between(-1, 1), random.between(-1, 1), 0};
        const Quaternion turned = (Quaternion::cayley(tilt) * aboutAxis({0, 0, 1}, random.between(0, 360))).unit();
        const Transform pa = at({}, aboutAxis({0, 0, 1}, random.between(0, 360)));
        const Transform pb = at({random.between(-0.35, 0.35), random.between(-0.35, 0.35), 0}, turned);
        const SeparatingAxis sat = findSeparatingAxis(wall, pa, plate, pb);
        const ClosestPoints gjk = closestPoints(wall, pa, plate, pb);
        falseGaps += sat.separation > 1e-9 && (gjk.coresOverlap || gjk.distance < sat.separation - 1e-9);
    }
    CHECK(falseGaps == 0, "разделяющая ось дала зазор больше расстояния в %d пробах из 5000", falseGaps);
}

// Почти параллельные грани двух брусков, вошедшие друг в друга на микрон. Точная ось наименьшего
// перекрытия здесь — пара рёбер, и она даёт одну точку; но касаются бруски всей площадкой, и
// контакт обязан её показать. С одной точкой удар останавливал сближение только в ней: два бруска
// на 100 м/с входили друг в друга на 36 см (docs/reports/case-table.md).
void contactPatchOfParallelFaces() {
    Sequence random;
    int edgeAxes = 0, singlePoints = 0;
    const ConvexHull brick = ConvexHull::box({0.25, 0.15, 0.2});
    for (int trial = 0; trial < 2000; ++trial) {
        const Vector3 tilt = std::pow(10.0, random.between(-6, -4.5)) * Vector3{random.between(-1, 1), 0, random.between(-1, 1)};
        Transform upper = at({random.between(-0.05, 0.05), 1, random.between(-0.05, 0.05)}, (Quaternion::cayley(tilt) * aboutAxis({0, 1, 0}, random.between(0, 360))).unit());
        upper.position.y -= findSeparatingAxis(brick, at({}), brick, upper).separation + 1e-6;
        if (findSeparatingAxis(brick, at({}), brick, upper).kind != SeparatingAxis::Kind::EdgePair) continue;
        ++edgeAxes;
        singlePoints += HullVsHullAlgorithm::clipFaces(brick, at({}), brick, upper, 1e-4, 1e-4).count < 3;
    }
    CHECK(edgeAxes > 100, "проверка не попала в нужный случай: ось пары рёбер только в %d пробах", edgeAxes);
    CHECK(singlePoints == 0, "площадка почти параллельных граней дала меньше трёх точек в %d пробах из %d", singlePoints, edgeAxes);
}

// --- Точки контакта --------------------------------------------------------------------------

// Площадь многоугольника из точек контакта, взятых в порядке обхода вокруг их центра.
double patchArea(const ContactManifold& m) {
    if (m.count < 3) return 0;
    Vector3 centre;
    for (int i = 0; i < m.count; ++i) centre += (1.0 / m.count) * m.points[i].onA;
    // Опорное направление в плоскости контакта и угол каждой точки относительно него.
    const Vector3 u = (1 / (m.points[0].onA - centre).length()) * (m.points[0].onA - centre), w = m.normal.cross(u);
    std::vector<std::pair<double, Vector3>> ordered;
    for (int i = 0; i < m.count; ++i) {
        const Vector3 r = m.points[i].onA - centre;
        ordered.push_back({std::atan2(r.dot(w), r.dot(u)), r});
    }
    std::sort(ordered.begin(), ordered.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    double doubled = 0;
    for (size_t i = 0; i < ordered.size(); ++i) doubled += ordered[i].second.cross(ordered[(i + 1) % ordered.size()].second).dot(m.normal);
    return std::fabs(doubled) / 2;
}

bool sameIds(const ContactManifold& a, const ContactManifold& b) {
    if (a.count != b.count) return false;
    for (int i = 0; i < a.count; ++i) {
        bool found = false;
        for (int j = 0; j < b.count; ++j) found = found || a.points[i].id == b.points[j].id;
        if (!found) return false;
    }
    return true;
}

void contactPoints() {
    const ConvexHull big = ConvexHull::box({1, 1, 1}), small = ConvexHull::box({0.5, 0.5, 0.5});
    const double depth = 0.01, margin = 0.02;

    // Малый куб стоит на большом, утоплен на depth: четыре точки — углы малой грани.
    const ContactManifold stacked = HullVsHullAlgorithm::clipFaces(big, at({}), small, at({0, 1.5 - depth, 0}), margin);
    CHECK(stacked.count == 4, "точек %d вместо 4", stacked.count);
    CHECK_NEAR((stacked.normal - Vector3{0, 1, 0}).length(), 0, kExact);
    for (int i = 0; i < stacked.count; ++i) {
        const ContactPoint& p = stacked.points[i];
        CHECK_NEAR(p.separation, -depth, kExact);
        CHECK_NEAR(std::fabs(p.onA.x), 0.5, kExact);
        CHECK_NEAR(std::fabs(p.onA.z), 0.5, kExact);
        CHECK_NEAR(p.onA.y, 1, kExact);
        CHECK_NEAR(p.onB.y, 1 - depth, kExact);
    }
    CHECK_NEAR(patchArea(stacked), 1.0, kExact);

    // Сдвинутые кубы: общая область граней — прямоугольник 1 x 1.5.
    const ContactManifold shifted = HullVsHullAlgorithm::clipFaces(big, at({}), big, at({1, 2 - depth, 0.5}), margin);
    CHECK(shifted.count == 4, "точек %d вместо 4", shifted.count);
    CHECK_NEAR(patchArea(shifted), 1.5, 1e-9);

    // Куб, повёрнутый на 45° вокруг вертикали: общая область — правильный восьмиугольник.
    // В контакт идут все восемь вершин: четыре точки не покрыли бы площадку (proofs/README.md).
    const ContactManifold turned = HullVsHullAlgorithm::clipFaces(big, at({}), big, at({0, 2 - depth, 0}, aboutAxis({0, 1, 0}, 45)), margin);
    const double octagon = 8 * (std::sqrt(2.0) - 1);
    CHECK(turned.count == 8, "точек %d вместо 8", turned.count);
    CHECK_NEAR(patchArea(turned), octagon, 1e-9);
    for (int i = 0; i < turned.count; ++i) CHECK_NEAR(turned.points[i].separation, -depth, 1e-9);

    // Куб, наклонённый на 10° и опирающийся ребром: дальняя сторона грани выше запаса, остаются две точки.
    const double lean = 10 * 3.14159265358979323846 / 180;
    const double lift = std::cos(lean) + std::sin(lean); // высота центра, при которой нижнее ребро на уровне грани
    const ContactManifold leaning = HullVsHullAlgorithm::clipFaces(big, at({}), big, at({0, 1 + lift - depth, 0}, aboutAxis({0, 0, 1}, 10)), margin);
    CHECK(leaning.count == 2, "точек %d вместо 2", leaning.count);
    for (int i = 0; i < leaning.count; ++i) CHECK_NEAR(leaning.points[i].separation, -depth, 1e-9);

    // Скрещённые рёбра (та же постановка, что в проверке разделяющей оси): одна точка на оси X.
    const double c = 2 * std::sqrt(2.0) - depth;
    const ContactManifold crossed = HullVsHullAlgorithm::clipFaces(big, at({}, aboutAxis({0, 0, 1}, 45)), big, at({c, 0, 0}, aboutAxis({0, 1, 0}, 45)), margin);
    CHECK(crossed.count == 1, "точек %d вместо 1", crossed.count);
    CHECK_NEAR((crossed.points[0].onA - Vector3{std::sqrt(2.0), 0, 0}).length(), 0, 1e-9);
    CHECK_NEAR((crossed.points[0].onB - Vector3{c - std::sqrt(2.0), 0, 0}).length(), 0, 1e-9);

    // Зазор меньше запаса: контакт будущий, расстояние положительное. Зазор больше запаса — пусто.
    const ContactManifold ahead = HullVsHullAlgorithm::clipFaces(big, at({}), small, at({0, 1.5 + 0.005, 0}), margin);
    CHECK(ahead.count == 4, "точек %d вместо 4", ahead.count);
    for (int i = 0; i < ahead.count; ++i) CHECK_NEAR(ahead.points[i].separation, 0.005, kExact);
    CHECK(HullVsHullAlgorithm::clipFaces(big, at({}), small, at({0, 1.5 + 0.05, 0}), margin).count == 0, "за пределами запаса контакта быть не должно");

    // Номера точек переживают шаг: чуть другая глубина и чуть другой сдвиг дают те же номера.
    CHECK(sameIds(stacked, HullVsHullAlgorithm::clipFaces(big, at({}), small, at({0.001, 1.5 - 1.1 * depth, -0.002}), margin)), "номера точек изменились");
    CHECK(sameIds(shifted, HullVsHullAlgorithm::clipFaces(big, at({}), big, at({1.001, 2 - 1.1 * depth, 0.499}), margin)), "номера точек изменились");
    // Грани, совпадающие по контуру: вершины лежат на границе отсечения. Номера не должны зависеть от шума.
    const ContactManifold flush = HullVsHullAlgorithm::clipFaces(big, at({}), big, at({0, 2 - depth, 0}), margin);
    CHECK(flush.count == 4, "точек %d вместо 4", flush.count);
    CHECK(sameIds(flush, HullVsHullAlgorithm::clipFaces(big, at({}), big, at({1e-12, 2 - depth, -1e-12}), margin)), "номера зависят от шума округления");
    for (int i = 0; i < flush.count; ++i)
        for (int j = i + 1; j < flush.count; ++j) CHECK(flush.points[i].id != flush.points[j].id, "два контакта с одним номером");
}

// Разбиение невыпуклого тела на выпуклые части и точные тела вращения.
void decomposition() {
    const auto volumeOf = [](const std::vector<std::vector<Vector3>>& parts) {
        double volume = 0;
        for (const std::vector<Vector3>& points : parts) volume += double(MassFrame::of(ConvexHull::fromPoints(points), 0).volume);
        return volume;
    };
    // На сколько точка снаружи всех частей: отрицательное — внутри какой-нибудь.
    const auto outside = [](const std::vector<std::vector<Vector3>>& parts, Vector3 point) {
        double least = 1e30;
        for (const std::vector<Vector3>& points : parts) {
            double worst = -1e30;
            for (const ConvexHull::Face& face : ConvexHull::fromPoints(points).faces()) worst = std::fmax(worst, double(face.normal.dot(point) - face.offset));
            least = std::fmin(least, worst);
        }
        return least;
    };
    {
        // Выпуклое тело не режется: повёрнутый на 45° брусок — одна часть того же объёма.
        const TriangleMesh brick = lathe({{0, 0}, {0.2, 0}, {0.2, 0.2}, {0, 0.2}}, 4);
        const std::vector<std::vector<Vector3>> parts = convexParts(brick);
        CHECK(parts.size() == 1, "выпуклый брусок разбит на %zu частей", parts.size());
        CHECK_NEAR(volumeOf(parts), double(brick.volume()), 1e-9);
    }
    {
        // Тор режется на дуги: отверстие остаётся открытым, поверхность — покрытой.
        const TriangleMesh torus = torusMesh(0.2, 0.05);
        const std::vector<std::vector<Vector3>> parts = convexParts(torus);
        CHECK(parts.size() >= 8 && parts.size() <= 16, "тор разбит на %zu частей", parts.size());
        CHECK(volumeOf(parts) > double(torus.volume()) && volumeOf(parts) < 1.06 * double(torus.volume()), "объём частей тора %.5f при объёме тора %.5f", volumeOf(parts), double(torus.volume()));
        // Диагональ тора — 0,71 м; допуск выступа в пустоту — воксель, 0,5 / 32 м.
        const double tolerance = 0.5 / 32;
        CHECK(outside(parts, {0, 0, 0}) > 0.15 - 2 * tolerance, "части тора закрывают отверстие: до центра %.4f м", outside(parts, {0, 0, 0}));
        double uncovered = 0;
        for (Vector3 v : torus.vertices) uncovered = std::fmax(uncovered, outside(parts, v));
        CHECK(uncovered < tolerance, "вершина тора в %.4f м снаружи частей", uncovered);
        for (const std::vector<Vector3>& points : parts) CHECK(points.size() <= 32, "у части тора %zu вершин", points.size());
        // Масса невыпуклого тела — плотность на объём сетки.
        const Body body = CompoundBodyBuilder::solid(torus, 800);
        CHECK_NEAR(1 / double(body.mass.inverseMass), 800 * double(torus.volume()), 1e-9);
        CHECK(body.partCount() == int(parts.size()), "у тела-тора частей %d", body.partCount());
    }
    {
        // Тела вращения точно: части в сумме дают гранёное тело, без лишнего объёма.
        const Body torus = CompoundBodyBuilder::torus(0.2, 0.05, 800);
        CHECK(torus.partCount() == 16, "у точного тора частей %d", torus.partCount());
        CHECK_NEAR(1 / double(torus.mass.inverseMass), 800 * double(torusMesh(0.2, 0.05, 16, 8).volume()), 1e-9);
        const Body bucket = CompoundBodyBuilder::bucket(0.15, 0.2, 0.3, 0.01, 800);
        CHECK(bucket.partCount() == 17, "у ведра частей %d", bucket.partCount());
        CHECK_NEAR(1 / double(bucket.mass.inverseMass), 800 * double(bucketMesh(0.15, 0.2, 0.3, 0.01, 16).volume()), 1e-9);
        // Центр масс ведра — на оси, ниже середины высоты (дно тяжелее пустоты).
        CHECK(std::fabs(double(bucket.position.x)) < 1e-9 && std::fabs(double(bucket.position.z)) < 1e-9 && double(bucket.position.y) > 0.05 && double(bucket.position.y) < 0.15,
              "центр масс ведра в (%.4f, %.4f, %.4f)", double(bucket.position.x), double(bucket.position.y), double(bucket.position.z));
    }
}

} // namespace

void geometryTests() {
    polytopeTopology();
    spheres();
    boxes();
    roundedShapes();
    separatingAxes();
    gjkAgreesWithSat();
    nearlyParallelFaces();
    boxesAboutCommonAxis();
    contactPatchOfParallelFaces();
    contactPoints();
    decomposition();
}
