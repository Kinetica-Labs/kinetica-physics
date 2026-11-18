// Зонд: какое отображение «вектор поворота -> кватернион» ставить в правило средней точки.
#include <kp/body/Body.h>
#include <kp/dynamics/Integrator.h>
#include <cmath>
#include <cstdio>
using namespace kp;

// Середина дуги между двумя ориентациями: для единичных кватернионов это нормированная сумма.
Quaternion midpoint(Quaternion a, Quaternion b) { return Quaternion{a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z}.unit(); }

Quaternion mapCayley(Vector3 t) { return Quaternion::cayley(t); }
// Кэли с поправкой: h -> h tg|h|/|h| рядом Тейлора, тогда угол поворота равен |θ| до O(θ^9).
Quaternion mapSeries(Vector3 t) {
    const Real s = 0.25 * t.dot(t);
    const Real k = 1 + s * (Real(1) / 3 + s * (Real(2) / 15 + s * Real(17) / 315));
    return Quaternion::cayley(k * t);
}
Quaternion mapExp(Vector3 t) {
    const double a = std::sqrt(t.dot(t));
    const double k = a < 1e-9 ? 0.5 : std::sin(0.5 * a) / a;
    return {std::cos(0.5 * a), k * t.x, k * t.y, k * t.z};
}
template <class Map> void integratePosition(Body& b, Real dt, Map map, int iterations = 4) {
    const Quaternion start = b.orientation;
    Quaternion end = start;
    for (int i = 0; i < iterations; ++i) end = (map(dt * b.angularVelocityAt(midpoint(start, end))) * start).unit();
    b.orientation = end;
}
double angleBetween(Quaternion a, Quaternion b) {
    const Quaternion d = a * b.conjugate();
    return 2 * std::atan2(std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z), std::fabs(d.w));
}
Body box(Vector3 w) {
    Body b;
    b.mass = MassProperties::box({0.1, 0.2, 0.4}, 1000);
    b.angularMomentum = {w.x / b.mass.inverseInertia.x, w.y / b.mass.inverseInertia.y, w.z / b.mass.inverseInertia.z};
    return b;
}
template <class Map> void study(const char* name, Map map) {
    std::printf("%s\n", name);
    for (double w : {5.0, 30.0, 100.0, 300.0}) { // вращение около главной оси: точный ответ — угол ω dt
        Body b = box({0, 0, w});
        integratePosition(b, 1.0 / 60, map);
        const double step = 2 * std::atan2(b.orientation.z, b.orientation.w);
        std::printf("  ось, ω=%g рад/с: отставание фазы %.3g град/с\n", w, (w / 60 - step) * 60 * 180 / M_PI);
    }
    for (double w : {5.0, 30.0}) { // кувырок около средней оси; эталон — то же правило с шагом в 256 раз мельче
        for (int substeps : {1, 2, 4}) {
            Body b = box({w, 0.2 * w, 0.2 * w}), ref = b;
            const double e0 = b.kineticEnergy();
            double worst = 0;
            const int frames = 600;
            for (int f = 0; f < frames; ++f) {
                for (int s = 0; s < substeps; ++s) integratePosition(b, 1.0 / 60 / substeps, map);
                for (int s = 0; s < 256; ++s) integratePosition(ref, 1.0 / 60 / 256, mapExp);
                worst = std::fmax(worst, std::fabs(b.kineticEnergy() - e0) / e0);
            }
            std::printf("  прецессия, ω=%g, подшагов %d: энергия %.3g, ориентация через 10 с %.3g град\n", w, substeps, worst,
                        angleBetween(b.orientation, ref.orientation) * 180 / M_PI);
        }
    }
}
// Интегратор движка, как он есть: отставание фазы шара при вращении вокруг оси.
void engineIntegrator() {
    std::printf("интегратор движка (integratePosition)\n");
    for (double w : {5.0, 30.0, 100.0, 300.0}) {
        Body b = Body::sphere(0.1, 1000);
        b.angularMomentum = {0, 0, w / b.mass.inverseInertia.z};
        integratePosition(b, 1.0 / 60);
        const double step = 2 * std::atan2(b.orientation.z, b.orientation.w);
        std::printf("  ось, ω=%g рад/с: отставание фазы %.3g град/с\n", w, (w / 60 - step) * 60 * 180 / M_PI);
    }
}

int main() {
    engineIntegrator();
    study("Кэли (сейчас)", mapCayley);
    study("Кэли + ряд Тейлора tg", mapSeries);
    study("экспонента (sin, cos)", mapExp);
}
