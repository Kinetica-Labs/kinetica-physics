#include <kp/dynamics/event_driven/OverlapSeparation.h>

#include <kp/dynamics/event_driven/ContactSolver.h>
#include <kp/dynamics/Integrator.h>

namespace kp {

bool OverlapSeparation::separate(const std::vector<ContactPair>& touching) {
    // Точки контакта меряют зазор от плоскости грани, а у её края тела входят друг в друга и
    // мимо этой плоскости: ребро оболочки вдавлено в бок цилиндра на 7,7 мкм, а точка на грани
    // показывает 4,8. По таким точкам раздвигание не срабатывало, хотя поиск начала перекрытия,
    // который смотрит на истинную глубину, уже не пускал тела дальше, — шаг вставал. Поэтому,
    // если истинная глубина больше той, что показывают точки, пара раздвигается по истинной
    // нормали на истинную глубину, в самой глубокой своей точке.
    std::vector<ContactPair> pushed = touching;
    bool overlapping = false;
    for (ContactPair& pair : pushed) {
        int deepest = -1;
        for (int i = 0; i < pair.manifold.count; ++i)
            if (deepest < 0 || pair.manifold.points[i].separation < pair.manifold.points[size_t(deepest)].separation) deepest = i;
        if (deepest >= 0 && pair.inContact()) {
            const Separation separation = separationOf(bodies_[size_t(pair.a)].part(pair.partA), bodies_[size_t(pair.b)].part(pair.partB));
            if (separation.distance < pair.manifold.points[size_t(deepest)].separation - kOverlap / 4) {
                ContactPoint point = pair.manifold.points[size_t(deepest)];
                point.separation = separation.distance;
                pair.manifold.points.clear();
                pair.manifold.add(point);
                pair.manifold.normal = separation.normal;
            }
        }
        for (int i = 0; i < pair.manifold.count; ++i) overlapping = overlapping || pair.manifold.points[size_t(i)].separation < -kOverlap / 2;
    }
    // Разошедшееся сочленение возвращается на место тем же способом: сдвигом поз без скорости.
    overlapping = overlapping || joints_.largestError(bodies_) > kOverlap / 2;
    if (!overlapping) return false;

    std::vector<Body> shadows = bodies_;
    for (Body& shadow : shadows) shadow.linearMomentum = shadow.angularMomentum = {};
    ContactSolver(shadows, pushed, up_, 0, nullptr, &joints_).solveOverlap();
    for (size_t i = 0; i < bodies_.size(); ++i) {
        integratePosition(shadows[i], 1);
        bodies_[i].position = shadows[i].position;
        bodies_[i].orientation = shadows[i].orientation;
    }
    return true;
}

} // namespace kp
