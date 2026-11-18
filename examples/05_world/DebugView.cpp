// Отладочный вывод целой сцены: шар сбивает стопку кубов. В консоль — состояние мира словами,
// в файл — картинка по кадрам: рёбра тел, точки контакта и нормали.

#include "HtmlViewer.h"

using namespace kp;

int main() {
    World world;
    world.addBody(Body::box({5, 0.5, 5}, 0, {0, -0.5, 0}));
    for (int k = 0; k < 3; ++k) world.addBody(Body::box({0.2, 0.2, 0.2}, 1000, {0, 0.2 + 0.4 * k, 0}));
    Body ball = Body::sphere(0.15, 1000, {-2, 0.9, 0});
    ball.linearMomentum = {5 / ball.mass.inverseMass, 0, 0}; // летит вправо со скоростью 5 м/с
    world.addBody(ball);

    HtmlViewer viewer({0, 0.6, 0}, 5);
    for (int frame = 0; frame < 180; ++frame) {
        world.step(Real(1) / 60);
        viewer.frame(debugGeometry(world));
        if (frame % 60 == 59) std::printf("--- t = %d с\n%s", (frame + 1) / 60, debugText(world).c_str());
    }
    viewer.save("DebugView.html");
    return 0;
}
