// Слой geometry: выпуклая оболочка облака точек. Так в движок попадает деталь произвольной формы:
// точки её поверхности «обтягиваются» многогранником, внутренние отбрасываются, а треугольники,
// лежащие в одной плоскости, сливаются в настоящие грани.

#include "HtmlViewer.h"

using namespace kp;

int main() {
    // Куб со стороной 2 м, заданный с избытком: сетка точек на каждой грани и точки внутри.
    std::vector<Vector3> points;
    for (int x = 0; x <= 4; ++x)
        for (int y = 0; y <= 4; ++y)
            for (int z = 0; z <= 4; ++z) points.push_back({-1 + 0.5 * x, -1 + 0.5 * y, -1 + 0.5 * z});
    const ConvexHull cube = ConvexHull::fromPoints(points);
    std::printf("куб из %zu точек: вершин %zu, граней %zu, рёбер %zu (должно быть 8, 6, 12)\n", points.size(), cube.vertices().size(),
                cube.faces().size(), cube.edges().size());

    // Срезанный угол: добавлена одна точка снаружи — у оболочки появляется вершина и новые грани.
    points.push_back({1.6, 1.6, 1.6});
    const ConvexHull pointed = ConvexHull::fromPoints(points);
    std::printf("с точкой снаружи: вершин %zu, граней %zu, рёбер %zu\n", pointed.vertices().size(), pointed.faces().size(), pointed.edges().size());

    // Тело из тех же точек: масса, центр масс и главные оси инерции находятся сами.
    const Body body = Body::hull(points, 1000);
    std::printf("масса %.1f кг, центр масс (%.4f, %.4f, %.4f)\n", 1 / body.mass.inverseMass, body.position.x, body.position.y, body.position.z);

    HtmlViewer viewer({0.2, 0.2, 0}, 6);
    viewer.frame(DebugGeometry().add(pointed, Transform{}));
    viewer.save("Hull.html");
    return 0;
}
