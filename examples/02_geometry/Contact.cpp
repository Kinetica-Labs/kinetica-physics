// Слой geometry: точки контакта двух брусков. Верхний лежит на нижнем со сдвигом и поворотом,
// поэтому пятно контакта — не квадрат, а обрезанный многоугольник; от него остаются четыре точки.

#include <kp/collision/narrowphase/HullVsHullAlgorithm.h>
#include "HtmlViewer.h"

using namespace kp;

int main() {
    const ConvexHull lower = ConvexHull::box({1, 0.5, 1}), upper = ConvexHull::box({0.5, 0.5, 0.5});
    // Верхний вошёл в нижний на 1 мм и повёрнут вокруг вертикали на 53° — кватернион (2, 0, 1, 0).
    const Transform lowerPose{}, upperPose{{0.8, 0.999, 0}, Quaternion{2, 0, 1, 0}.unit()};

    const ContactManifold manifold = HullVsHullAlgorithm::clipFaces(lower, lowerPose, upper, upperPose, 0.01);

    std::printf("нормаль (%.3f, %.3f, %.3f), точек %d\n", manifold.normal.x, manifold.normal.y, manifold.normal.z, manifold.count);
    for (int k = 0; k < manifold.count; ++k) {
        const ContactPoint& point = manifold.points[k];
        std::printf("  точка (%.3f, %.3f, %.3f), зазор %.4f м, номер %08x\n", point.onA.x, point.onA.y, point.onA.z,
                    point.separation, unsigned(point.id));
    }

    HtmlViewer viewer({0.4, 0.6, 0}, 4);
    viewer.frame(DebugGeometry().add(lower, lowerPose, DebugKind::StaticShape).add(upper, upperPose).add(manifold));
    viewer.save("Contact.html");
    return 0;
}
