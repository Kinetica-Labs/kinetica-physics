// Слой dynamics: свободное вращение. Брусок раскручен около средней оси и раз в несколько секунд
// сам переворачивается (эффект Джанибекова). Сил нет, значит момент импульса и энергия обязаны
// сохраняться — по ним видно качество интегратора.

#include "HtmlViewer.h"
#include <dynamics/Integrator.h>

using namespace kp;

int main() {
    Body body = Body::box({0.1, 0.2, 0.4}, 1000);
    // Момент импульса L = I ω: 5 рад/с около оси Y и малый толчок по двум другим.
    const Vector3 spin{0.01, 5, 0.01}, inverseInertia = body.mass.inverseInertia;
    body.angularMomentum = {spin.x / inverseInertia.x, spin.y / inverseInertia.y, spin.z / inverseInertia.z};
    const Real energy = kineticEnergyOf(body);

    HtmlViewer viewer({}, 2);
    for (int frame = 1; frame <= 600; ++frame) {
        integratePosition(body, Real(1) / 60);
        viewer.frame(DebugGeometry().add(*body.shape, body.pose()));
        if (frame % 60 != 0) continue;
        const Vector3 axis = body.pose().direction({0, 1, 0}), L = body.angularMomentum;
        std::printf("t = %2d с: ось Y тела смотрит в (%+.2f, %+.2f, %+.2f), энергия изменилась на %+.1e, L = (%.6f, %.6f, %.6f)\n",
                    frame / 60, axis.x, axis.y, axis.z, kineticEnergyOf(body) / energy - 1, L.x, L.y, L.z);
    }
    viewer.save("FreeSpin.html");
    return 0;
}
