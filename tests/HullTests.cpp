// Проверки построителя выпуклой оболочки (collision/shapes/QuickHull.cpp).

#include "Check.h"

#include <body/Body.h>
#include <body/MassFrame.h>
#include <collision/ContactPair.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

using namespace kp;

namespace {

double uniform(std::uint64_t& state) { // детерминированный генератор, [0, 1)
    state = state * 6364136223846793005ull + 1442695040888963407ull;
    return double(state >> 11) / double(1ull << 53);
}

// Оболочка верна, если: все исходные точки внутри; грани плоские и выпуклые; формула Эйлера
// В − Р + Г = 2 выполняется. tolerance — допуск на расстояния, м.
void checkHull(const char* name, const ConvexHull& hull, const std::vector<Vector3>& points, double tolerance) {
    const ConvexShape& shape = hull;
    const Vector3 extent = shape.coreExtents();
    for (const Vector3 axis : {Vector3{1,0,0}, Vector3{0,1,0}, Vector3{0,0,1}}) {
        const Real expected = std::max(axis.dot(hull.support(axis)), -axis.dot(hull.support(-axis)));
        CHECK_NEAR(axis.dot(extent), expected, 0);
    }
    double worstOutside = 0, worstFlatness = 0;
    bool convex = true;
    for (const ConvexHull::Face& face : hull.faces()) {
        for (Vector3 p : points) worstOutside = std::fmax(worstOutside, face.normal.dot(p) - face.offset);
        const size_t n = face.loop.size();
        for (size_t i = 0; i < n; ++i) {
            const Vector3 a = hull.vertices()[size_t(face.loop[i])], b = hull.vertices()[size_t(face.loop[(i + 1) % n])],
                       c = hull.vertices()[size_t(face.loop[(i + 2) % n])];
            worstFlatness = std::fmax(worstFlatness, std::fabs(face.normal.dot(a) - face.offset));
            convex = convex && face.normal.dot((b - a).cross(c - b)) > 0; // поворот против часовой стрелки, без прямых углов в 180°
        }
    }
    CHECK(worstOutside <= tolerance, "%s: точка снаружи оболочки на %.3e", name, worstOutside);
    CHECK(worstFlatness <= tolerance, "%s: грань неплоская на %.3e", name, worstFlatness);
    CHECK(convex, "%s: у грани есть вогнутый или развёрнутый угол", name);
    const int euler = int(hull.vertices().size()) - int(hull.edges().size()) + int(hull.faces().size());
    CHECK(euler == 2, "%s: В − Р + Г = %d (вершин %zu, рёбер %zu, граней %zu)", name, euler, hull.vertices().size(), hull.edges().size(),
          hull.faces().size());
}

void checkCounts(const char* name, const ConvexHull& hull, size_t vertices, size_t faces, size_t edges) {
    CHECK(hull.vertices().size() == vertices && hull.faces().size() == faces && hull.edges().size() == edges,
          "%s: вершин %zu, граней %zu, рёбер %zu; ожидалось %zu, %zu, %zu", name, hull.vertices().size(), hull.faces().size(),
          hull.edges().size(), vertices, faces, edges);
}

// Куб, заданный с избытком: углы, точки внутри, сетка точек на каждой грани и на рёбрах.
// Оболочка обязана вернуть ровно куб: 8 вершин, 6 четырёхугольных граней, 12 рёбер.
std::vector<Vector3> cubeCloud(std::uint64_t& random) {
    std::vector<Vector3> points;
    for (int i = 0; i < 300; ++i) points.push_back({2 * uniform(random) - 1, 2 * uniform(random) - 1, 2 * uniform(random) - 1});
    for (int axis = 0; axis < 3; ++axis)
        for (double side : {-1.0, 1.0})
            for (int u = 0; u <= 4; ++u)
                for (int v = 0; v <= 4; ++v) {
                    const double p[3] = {side, -1 + 0.5 * u, -1 + 0.5 * v};
                    points.push_back({p[axis], p[(axis + 1) % 3], p[(axis + 2) % 3]});
                }
    return points;
}

void cubeFromRedundantPoints() {
    std::uint64_t random = 11;
    const std::vector<Vector3> points = cubeCloud(random);
    const ConvexHull hull = ConvexHull::fromPoints(points);
    checkHull("куб с избытком", hull, points, 1e-9);
    checkCounts("куб с избытком", hull, 8, 6, 12);
    for (const ConvexHull::Face& face : hull.faces()) CHECK(face.loop.size() == 4, "куб: грань из %zu вершин", face.loop.size());

    // Тот же куб, повёрнутый так, что грани не идут вдоль осей.
    std::vector<Vector3> turned;
    const Quaternion q = Quaternion{1, Real(0.3), Real(-0.5), Real(0.7)}.unit();
    for (Vector3 p : points) turned.push_back(q.rotate(p));
    const ConvexHull turnedHull = ConvexHull::fromPoints(turned);
    checkHull("повёрнутый куб", turnedHull, turned, 1e-9);
    checkCounts("повёрнутый куб", turnedHull, 8, 6, 12);
}

// Призма с двенадцатиугольными основаниями: 24 вершины, 12 боковых граней и 2 основания, 36 рёбер.
// Основание триангулируется десятью треугольниками, и все они обязаны слиться в одну грань.
void prism() {
    std::vector<Vector3> points;
    Real x = 1, z = 0;
    const Real cos30 = Real(0.86602540378443865), sin30 = Real(0.5);
    for (int k = 0; k < 12; ++k) {
        points.push_back({x, -1, z});
        points.push_back({x, 1, z});
        points.push_back({Real(0.5) * x, 0, Real(0.5) * z}); // точка внутри
        const Real nextX = cos30 * x - sin30 * z;
        z = sin30 * x + cos30 * z;
        x = nextX;
    }
    const ConvexHull hull = ConvexHull::fromPoints(points);
    checkHull("призма", hull, points, 1e-9);
    checkCounts("призма", hull, 24, 14, 36);
}

// Случайные облака и точки на сфере. У точек на сфере все лежат на оболочке, грани — треугольники:
// граней 2n − 4, рёбер 3n − 6.
void randomClouds() {
    std::uint64_t random = 5;
    for (int count : {4, 5, 8, 20, 100, 500, 2000}) {
        std::vector<Vector3> cloud, sphere;
        for (int i = 0; i < count; ++i) {
            cloud.push_back({3 * uniform(random), uniform(random), 2 * uniform(random)});
            Vector3 direction;
            do direction = {2 * uniform(random) - 1, 2 * uniform(random) - 1, 2 * uniform(random) - 1};
            while (direction.length() < 0.1);
            sphere.push_back((1 / direction.length()) * direction);
        }
        checkHull("облако", ConvexHull::fromPoints(cloud), cloud, 1e-9);
        const ConvexHull ball = ConvexHull::fromPoints(sphere);
        checkHull("сфера", ball, sphere, 1e-9);
        checkCounts("сфера", ball, size_t(count), size_t(2 * count - 4), size_t(3 * count - 6));
    }
}

void flatCloudIsRejected() {
    bool rejected = false;
    try {
        ConvexHull::fromPoints({{0, 0, 0}, {1, 0, 0}, {0, 0, 1}, {1, 0, 1}, {0.3, 0, 0.2}});
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    CHECK(rejected, "плоское облако точек не отвергнуто");
}

void offsetCoreExtents() {
    const std::vector<Vector3> points{{3,-4,5}, {4,-4,5}, {3,-2,5}, {3,-4,8}};
    auto hull = std::make_shared<ConvexHull>(ConvexHull::fromPoints(points));
    const ConvexShape& shape = *hull;
    Body body = Body::box({1,2,3}, 1);
    body.shape = hull;
    body.orientation = Quaternion{1, 2, -1, 3}.unit();
    for (Real skin : {Real(0), Real(.125)}) {
        hull->radius = skin;
        const Vector3 extent = shape.coreExtents();
        CHECK_NEAR(extent.x, 4, 0);
        CHECK_NEAR(extent.y, 4, 0);
        CHECK_NEAR(extent.z, 8, 0);
    }
    hull->radius = 0;
    CHECK_NEAR(maxPointSpeed(body), 0, 0);
    body.angularMomentum = {3, -7, 1};
    for (Vector3 p : points) {
        const Real speed = pointVelocity(body, body.pose().point(p)).length();
        CHECK(speed <= maxPointSpeed(body), "скорость вершины %.12g м/с выше оценки", double(speed));
    }
}

// Масса и моменты инерции оболочки сверяются с формулами бруска; затем тот же брусок задаётся
// повёрнутым и сдвинутым — тело обязано получиться тем же, а его вершины в мире — на своих местах.
void massOfBox() {
    const Vector3 h{0.3, 0.2, 0.1};
    std::vector<Vector3> corners;
    for (int i = 0; i < 8; ++i) corners.push_back({i & 1 ? h.x : -h.x, i & 2 ? h.y : -h.y, i & 4 ? h.z : -h.z});
    const MassProperties expected = MassProperties::box(h, 800);
    const Body straight = Body::hull(corners, 800);
    CHECK_NEAR(straight.mass.inverseMass, expected.inverseMass, 1e-12 * expected.inverseMass);
    CHECK_NEAR(straight.mass.inverseInertia.x, expected.inverseInertia.x, 1e-12 * expected.inverseInertia.x);
    CHECK_NEAR(straight.mass.inverseInertia.y, expected.inverseInertia.y, 1e-12 * expected.inverseInertia.y);
    CHECK_NEAR(straight.mass.inverseInertia.z, expected.inverseInertia.z, 1e-12 * expected.inverseInertia.z);
    CHECK_NEAR(straight.position.length(), 0, 1e-15);

    const Quaternion turn = Quaternion{1, Real(0.3), Real(-0.5), Real(0.7)}.unit();
    const Vector3 shift{5, -3, 2};
    std::vector<Vector3> moved;
    for (Vector3 p : corners) moved.push_back(shift + turn.rotate(p));
    const Body turned = Body::hull(moved, 800, {10, 0, 0});
    CHECK_NEAR(turned.mass.inverseMass, expected.inverseMass, 1e-12 * expected.inverseMass);
    CHECK_NEAR((turned.position - (Vector3{10, 0, 0} + shift)).length(), 0, 1e-12); // центр масс
    // Главные моменты те же, в каком бы порядке ни нашлись оси.
    const Vector3 found = turned.mass.inverseInertia;
    CHECK_NEAR(found.x + found.y + found.z, expected.inverseInertia.x + expected.inverseInertia.y + expected.inverseInertia.z,
               1e-10 * expected.inverseInertia.z);
    CHECK_NEAR(found.x * found.y * found.z, expected.inverseInertia.x * expected.inverseInertia.y * expected.inverseInertia.z,
               1e-9 * expected.inverseInertia.x * expected.inverseInertia.y * expected.inverseInertia.z);
    // Каждая заданная точка — вершина тела в мире.
    const auto* shape = dynamic_cast<const ConvexHull*>(turned.shape.get());
    double worst = 0;
    for (Vector3 p : moved) {
        double nearest = 1e9;
        for (Vector3 v : shape->vertices()) nearest = std::fmin(nearest, (turned.pose().point(v) - (Vector3{10, 0, 0} + p)).length());
        worst = std::fmax(worst, nearest);
    }
    CHECK(worst < 1e-12, "вершина тела не на своём месте: %.3e м", worst);
}

} // namespace

void hullTests() {
    offsetCoreExtents();
    cubeFromRedundantPoints();
    prism();
    randomClouds();
    flatCloudIsRejected();
    massOfBox();
}
