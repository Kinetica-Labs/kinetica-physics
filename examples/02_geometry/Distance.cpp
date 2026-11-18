// Слой geometry: расстояние между двумя выпуклыми формами и ближайшие точки на них (алгоритм GJK).

#include <kp/collision/shapes/Sphere.h>
#include "HtmlViewer.h"

using namespace kp;

int main() {
    const ConvexHull box = ConvexHull::box({1, 0.5, 0.5}); // брусок 2 × 1 × 1 м
    const Sphere ball(0.5);                                 // шар радиуса 0,5 м
    const Transform boxPose{}, ballPose{{3, 2, 0}, {}};

    const ClosestPoints closest = closestPoints(box, boxPose, ball, ballPose);

    // Ближайшее к шару место бруска — ребро x = 1, y = 0,5. От него до центра шара
    // √(2² + 1,5²) = 2,5 м; минус радиус — 2 м.
    std::printf("расстояние %.6f м, точный ответ 2 м\n", closest.distance);
    std::printf("точка на бруске (%.3f, %.3f, %.3f)\n", closest.onA.x, closest.onA.y, closest.onA.z);
    std::printf("точка на шаре   (%.3f, %.3f, %.3f)\n", closest.onB.x, closest.onB.y, closest.onB.z);

    HtmlViewer viewer({1.5, 1, 0}, 8);
    viewer.frame(DebugGeometry().add(box, boxPose).add(ball, ballPose).add(closest));
    viewer.save("Distance.html");
    return 0;
}
