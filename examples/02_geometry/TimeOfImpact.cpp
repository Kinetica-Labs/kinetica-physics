// Слой geometry: время удара. Шар летит на плиту толщиной 1 мм со скоростью 1 км/с. За один кадр
// он пролетел бы 16,7 м — плиту при обычной проверке «пересеклись ли тела» он бы не заметил.
//
// Метод безопасных шагов: если до препятствия d метров, а скорость сближения не больше v,
// то d / v секунд можно лететь, не опасаясь столкновения. Шаги повторяются до касания.

#include <kp/collision/shapes/Sphere.h>
#include <kp/collision/shapes/ConvexHull.h>
#include <kp/collision/narrowphase/Gjk.h>
#include <kp/collision/narrowphase/TimeOfImpact.h>

#include <cstdio>

using namespace kp;

int main() {
    const ConvexHull plate = ConvexHull::box({0.0005, 1, 1});
    const Sphere ball(0.1);
    const Real speed = 1000, start = -1, frame = Real(1) / 60, touching = 1e-4;

    const auto distanceAt = [&](Real time) {
        return closestPoints(plate, Transform{}, ball, Transform{{start + speed * time, 0, 0}, {}}).distance;
    };
    const Real time = timeOfImpact(distanceAt, speed, frame, touching);

    // Точный ответ: зазор 1 − 0,1 − 0,0005 м, делённый на скорость. Найденный момент чуть раньше:
    // тела останавливаются внутри полосы касания шириной 0,1 мм, не доходя до перекрытия.
    std::printf("удар через %.9f с, точный ответ %.9f с\n", time, (1 - 0.1 - 0.0005) / speed);
    std::printf("в этот момент до плиты %.2e м\n", distanceAt(time));
    return 0;
}
