#include "ExactScenes.h"

#include <cmath>

using namespace kp;

namespace {

constexpr double kG = 9.81;
constexpr double kPiD = 3.14159265358979323846;
const Vector3 kPivot{0, 3, 0};

// Опора без формы: неподвижна и ни с чем не сталкивается.
BodyId anchor(World& world) { return world.addBody(Body{}); }

double massOf(const Body& body) { return 1 / double(body.mass.inverseMass); }

// Тело вращается как целое с угловой скоростью omega вокруг оси через точку about.
void spin(Body& body, Vector3 omega, Vector3 about) {
    const Vector3 inertia{1 / body.mass.inverseInertia.x, 1 / body.mass.inverseInertia.y, 1 / body.mass.inverseInertia.z};
    body.angularMomentum = body.orientation.rotate(inertia.scaled(body.orientation.rotateInverse(omega)));
    body.linearMomentum = Real(massOf(body)) * omega.cross(body.position - about);
}

double energy(const World& world) {
    double total = 0;
    for (const Body& body : world.bodies())
        if (!body.isStatic()) total += double(kineticEnergyOf(body)) + massOf(body) * kG * double(body.position.y);
    return total;
}

// --- Физический маятник: брусок на шарнире, отпущен под углом 120° к вертикали ------------------
// θ'' = −(m g d / I) sin θ, I — момент инерции относительно оси шарнира. Точное решение —
// эллиптическая функция; здесь то же уравнение одной свободы проинтегрировано методом
// Рунге — Кутты с шагом 10 мкс, его ошибка на шесть порядков меньше ошибки шага мира.
constexpr double kRodHalf = 0.4, kRelease = 2 * kPiD / 3;

Body rodAt(double angle, Vector3 pivot) {
    Body rod = Body::box({Real(kRodHalf), Real(0.03), Real(0.03)}, 1000);
    rod.orientation = Quaternion::fromRotationVector({0, 0, Real(angle - kPiD / 2)});
    rod.position = pivot + Real(kRodHalf) * Vector3{Real(std::sin(angle)), Real(-std::cos(angle)), 0};
    return rod;
}

World pendulum() {
    World world;
    const BodyId base = anchor(world);
    addHinge(world, base, world.addBody(rodAt(kRelease, kPivot)), kPivot, {0, 0, 1});
    return world;
}

double pendulumAngle(const World& world) {
    const Vector3 r = world.bodies()[1].position - kPivot;
    return std::atan2(double(r.x), double(-r.y));
}

double pendulumExact(double time) {
    static double at = 0, angle = kRelease, speed = 0;
    if (time < at) at = 0, angle = kRelease, speed = 0;
    // I / (m d) = (hx² + hy²) / (3 d) + d для бруска с полуразмерами hx, hy.
    const double k = kG / ((kRodHalf * kRodHalf + 0.03 * 0.03) / (3 * kRodHalf) + kRodHalf), dt = 1e-5;
    for (; at < time - dt / 2; at += dt) {
        const double a1 = -k * std::sin(angle), v2 = speed + dt / 2 * a1, a2 = -k * std::sin(angle + dt / 2 * speed);
        const double v3 = speed + dt / 2 * a2, a3 = -k * std::sin(angle + dt / 2 * v2), v4 = speed + dt * a3, a4 = -k * std::sin(angle + dt * v3);
        angle += dt / 6 * (speed + 2 * v2 + 2 * v3 + v4), speed += dt / 6 * (a1 + 2 * a2 + 2 * a3 + a4);
    }
    return angle;
}

// --- Цепь из десяти звеньев с грузом в сто раз тяжелее звена: энергия сохраняется --------------
World heavyChain() {
    World world;
    BodyId previous = anchor(world);
    for (int i = 0; i < 10; ++i) {
        Body link = Body::box({Real(0.1), Real(0.03), Real(0.03)}, i == 9 ? 100000 : 1000, kPivot + Vector3{Real(0.1 + 0.2 * i), 0, 0});
        const BodyId id = world.addBody(link);
        addHinge(world, previous, id, kPivot + Vector3{Real(0.2 * i), 0, 0}, {0, 0, 1});
        previous = id;
    }
    return world;
}

double chainEnergy(double) {
    static const double start = energy(heavyChain());
    return start;
}

// --- Гироскоп: диск на шаровой опоре, ось горизонтальна ------------------------------------------
// Диск вращается вокруг своей оси со скоростью ω₃ и не падает: ось обходит вертикаль со
// скоростью Ω = m g d / (I₃ ω₃). При горизонтальной оси это решение точное, если начать с него.
constexpr double kGyroArm = 0.3, kGyroRadius = 0.2, kGyroSpin = 60;
constexpr double kGyroTurn = kG * kGyroArm / (kGyroRadius * kGyroRadius / 2 * kGyroSpin);

World gyroscope() {
    World world;
    const BodyId base = anchor(world);
    Body disc = Body::cylinder(Real(0.03), Real(kGyroRadius), 1000, kPivot + Vector3{Real(kGyroArm), 0, 0});
    disc.orientation = Quaternion::fromRotationVector({0, 0, Real(-kPiD / 2)}); // ось диска — вдоль x
    spin(disc, {Real(kGyroSpin), Real(kGyroTurn), 0}, kPivot);
    addBallJoint(world, base, world.addBody(disc), kPivot);
    return world;
}

double offsetX(const World& world) { return double(world.bodies()[1].position.x - kPivot.x); }
double gyroscopeExact(double time) { return kGyroArm * std::cos(kGyroTurn * time); }

// --- Конический маятник: брусок на шаровой опоре описывает конус --------------------------------
// Брусок отклонён на β и вращается как целое вокруг вертикали: Ω² = m g d / ((I₁ − I₃) cos β),
// I₁ и I₃ — моменты инерции поперёк и вдоль бруска относительно опоры.
constexpr double kConeTilt = kPiD / 4;
double coneTurn() {
    const double across = (kRodHalf * kRodHalf + 0.03 * 0.03) / 3 + kRodHalf * kRodHalf, along = 2 * 0.03 * 0.03 / 3;
    return std::sqrt(kG * kRodHalf / ((across - along) * std::cos(kConeTilt)));
}

World conicalPendulum() {
    World world;
    const BodyId base = anchor(world);
    Body rod = rodAt(kConeTilt, kPivot);
    spin(rod, {0, Real(coneTurn()), 0}, kPivot);
    addBallJoint(world, base, world.addBody(rod), kPivot);
    return world;
}

double coneExact(double time) { return kRodHalf * std::sin(kConeTilt) * std::cos(coneTurn() * time); }

// --- Кривошип, шатун и поршень: замкнутый механизм с приводом -----------------------------------
// Кривошип длиной r вращается приводом со скоростью ω; поршень на ползуне ходит по закону
// x = r cos ωt + √(l² − r² sin² ωt). Шатун держится на двух шаровых опорах: так у механизма нет
// лишних связей.
constexpr double kCrank = 0.2, kRod = 0.6, kCrankSpeed = 4;

World sliderCrank() {
    World world;
    const BodyId base = anchor(world);
    const Vector3 tip = kPivot + Vector3{Real(kCrank), 0, 0}, pin = tip + Vector3{Real(kRod), 0, 0};
    Body crank = Body::box({Real(kCrank / 2), Real(0.02), Real(0.02)}, 1000, kPivot + Vector3{Real(kCrank / 2), 0, 0});
    spin(crank, {0, 0, Real(kCrankSpeed)}, kPivot);
    Body rod = Body::box({Real(kRod / 2), Real(0.02), Real(0.02)}, 1000, tip + Vector3{Real(kRod / 2), 0, 0});
    spin(rod, {0, 0, Real(-kCrankSpeed * kCrank / kRod)}, pin); // в этот миг шатун поворачивается вокруг пальца поршня
    const BodyId crankId = world.addBody(crank), rodId = world.addBody(rod), piston = world.addBody(Body::box({Real(0.06), Real(0.05), Real(0.05)}, 1000, pin));
    HingeJoint& drive = addHinge(world, base, crankId, kPivot, {0, 0, 1});
    drive.holdSpeed = true, drive.driveSpeed = Real(kCrankSpeed);
    addBallJoint(world, crankId, rodId, tip);
    addBallJoint(world, rodId, piston, pin);
    addSlider(world, base, piston, pin, {1, 0, 0});
    return world;
}

double pistonX(const World& world) { return double(world.bodies()[3].position.x - kPivot.x); }
double pistonExact(double time) {
    const double s = kCrank * std::sin(kCrankSpeed * time);
    return kCrank * std::cos(kCrankSpeed * time) + std::sqrt(kRod * kRod - s * s);
}

// --- Балка: шесть брусков, приваренных друг к другу и к опоре ------------------------------------
// Сварка жёсткая: конец балки обязан остаться на высоте опоры.
World weldedBeam() {
    World world;
    BodyId previous = anchor(world);
    for (int i = 0; i < 6; ++i) {
        const BodyId link = world.addBody(Body::box({Real(0.1), Real(0.03), Real(0.03)}, 1000, kPivot + Vector3{Real(0.1 + 0.2 * i), 0, 0}));
        addWeld(world, previous, link, kPivot + Vector3{Real(0.2 * i), 0, 0});
        previous = link;
    }
    return world;
}

double beamTip(const World& world) { return double(world.bodies()[6].position.y); }
double beamExact(double) { return double(kPivot.y); }

} // namespace

const std::vector<ExactScene>& exactScenes() {
    static const std::vector<ExactScene> all = {
        {"pendulum", "точное: маятник, размах 120°", pendulum, {0, 2.7, 0}, 3.5, "угол маятника, рад", pendulumAngle, pendulumExact},
        {"gyroscope", "точное: гироскоп на шаровой опоре", gyroscope, {0, 3, 0}, 2.5, "x центра диска, м", offsetX, gyroscopeExact},
        {"cone", "точное: конический маятник", conicalPendulum, {0, 2.7, 0}, 3, "x центра бруска, м", offsetX, coneExact},
        {"crank", "точное: кривошип, шатун, поршень", sliderCrank, {0.4, 3, 0}, 3, "x поршня, м", pistonX, pistonExact},
        {"chain100", "точное: цепь, груз 100:1", heavyChain, {0.5, 2, 0}, 6, "энергия, Дж", energy, chainEnergy},
        {"beam", "точное: балка из шести брусков", weldedBeam, {0.6, 2.9, 0}, 3.5, "высота конца балки, м", beamTip, beamExact},
    };
    return all;
}
