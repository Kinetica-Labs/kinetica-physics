#include <body/CompoundBodyBuilder.h>
#include <collision/ContactPair.h>
#include <collision/shapes/Meshes.h>
#include "StandardScenes.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

using namespace kp;

namespace {

std::string text(const char* format, ...) {
    char buffer[256];
    va_list arguments;
    va_start(arguments, format);
    std::vsnprintf(buffer, sizeof buffer, format, arguments);
    va_end(arguments);
    return buffer;
}

// Поворот на угол вокруг единичной оси. Сцены строятся обычной тригонометрией: правило «без
// libm» касается ядра.
Quaternion about(Vector3 axis, double degrees) {
    const double half = degrees * 3.14159265358979323846 / 360;
    return {Real(sinCos(half).cos), Real(sinCos(half).sin) * axis.x, Real(sinCos(half).sin) * axis.y, Real(sinCos(half).sin) * axis.z};
}

World withFloor() {
    World world;
    world.addBody(Body::box({10, 0.5, 10}, 0, {0, -0.5, 0}));
    return world;
}

// Тело не должно появляться внутри другого: начальное перекрытие шаг потом выталкивает, а в мерку
// перекрытия сцены оно попадало как наибольшее значение (куча без тяжести — 117 мм ещё до первого шага).
bool overlapsPlaced(const World& world, const Body& body) {
    for (const Body& other : world.bodies())
        if (!other.isStatic() && overlapDepth(other, body) > 0) return true;
    return false;
}

double speedOf(const Body& body) { return double(body.linearVelocity().length()) + double(body.angularVelocity().length()) * double(body.boundingRadius()); }

// Общая проверка сцен покоя: ни одно тело не сдвинулось дальше допуска и не движется.
std::string stayed(const World& start, const World& now, double tolerance) {
    for (size_t i = 0; i < now.bodies().size(); ++i) {
        const double moved = double((now.bodies()[i].position - start.bodies()[i].position).length());
        if (moved > tolerance) return text("тело %zu сдвинулось на %.1f мм", i, 1000 * moved);
        if (!now.bodies()[i].isStatic() && speedOf(now.bodies()[i]) > 1e-3) return text("тело %zu движется со скоростью %.1f мм/с", i, 1000 * speedOf(now.bodies()[i]));
    }
    return "";
}

// --- стопка из десяти кубов -------------------------------------------------------------------
World stack() {
    World world = withFloor();
    for (int k = 0; k < 10; ++k) world.addBody(Body::box({0.2, 0.2, 0.2}, 1000, {0, 0.2 + 0.4 * k, 0}));
    return world;
}
std::string stackVerdict(const World& start, const World& now) { return stayed(start, now, 1e-3); }

// --- пирамида: в основании восемь кубов, всего тридцать шесть ----------------------------------
World pyramid() {
    World world = withFloor();
    for (int row = 0; row < 8; ++row)
        for (int k = 0; k < 8 - row; ++k) world.addBody(Body::box({0.2, 0.2, 0.2}, 1000, {0.4 * k + 0.2 * row - 1.4, 0.2 + 0.4 * row, 0}));
    return world;
}
std::string pyramidVerdict(const World& start, const World& now) { return stayed(start, now, 1e-3); }

// --- тяжёлое на лёгком: куб в тонну на пяти кубах по килограмму ----------------------------------
World heavyOnLight() {
    World world = withFloor();
    for (int k = 0; k < 5; ++k) world.addBody(Body::box({0.2, 0.2, 0.2}, 1 / 0.064, {0, 0.2 + 0.4 * k, 0}));
    world.addBody(Body::box({0.2, 0.2, 0.2}, 1000 / 0.064, {0, 2.2, 0}));
    return world;
}
std::string heavyVerdict(const World& start, const World& now) { return stayed(start, now, 1e-3); }

// --- арка: девять клиньев держатся трением и формой ---------------------------------------------
World arch() {
    World world = withFloor();
    const double inner = 0.8, outer = 1.1, depth = 0.15;
    for (int k = 0; k < 9; ++k) {
        std::vector<Vector3> wedge;
        for (int end = 0; end < 2; ++end) {
            const double angle = (k + end) * 3.14159265358979323846 / 9;
            for (double radius : {inner, outer})
                for (double z : {-depth, depth}) wedge.push_back({Real(radius * sinCos(angle).cos), Real(radius * sinCos(angle).sin), Real(z)});
        }
        world.addBody(Body::hull(wedge, 2000));
    }
    return world;
}
std::string archVerdict(const World& start, const World& now) { return stayed(start, now, 2e-3); }

// --- склон: три бруска с трением 0,2, 0,5 и 0,8 на наклоне 30° ----------------------------------
// tg 30° = 0,577: первые два скользят с ускорением g (sin θ − μ cos θ), третий стоит.
const double kSlope = 30 * 3.14159265358979323846 / 180;
World slope() {
    World world;
    Body ramp = Body::box({6, 0.5, 3}, 0);
    ramp.orientation = about({0, 0, 1}, -30);
    ramp.position = ramp.orientation.rotate({0, -0.5, 0});
    ramp.friction = 1; // трение пары — среднее геометрическое, поэтому у бруска оно возводится в квадрат
    world.addBody(ramp);
    const double friction[3] = {0.2, 0.5, 0.8};
    for (int k = 0; k < 3; ++k) {
        Body block = Body::box({0.2, 0.1, 0.2}, 1000);
        block.orientation = ramp.orientation;
        block.position = ramp.orientation.rotate({-4, Real(0.1), Real(k - 1)});
        block.friction = Real(friction[k] * friction[k]);
        world.addBody(block);
    }
    return world;
}
std::string slopeVerdict(const World& start, const World& now) {
    const double friction[3] = {0.2, 0.5, 0.8}, time = 1;
    for (int k = 0; k < 3; ++k) {
        const double expected = std::max(0.0, 9.81 * (sinCos(kSlope).sin - friction[k] * sinCos(kSlope).cos)) * time * time / 2;
        const double travelled = double((now.bodies()[size_t(k + 1)].position - start.bodies()[size_t(k + 1)].position).length());
        if (std::fabs(travelled - expected) > 1e-3 + 1e-3 * expected) return text("брусок с трением %.1f прошёл %.4f м вместо %.4f", friction[k], travelled, expected);
    }
    return "";
}

// --- домино: двадцать костяшек, первая толкнута --------------------------------------------------
World dominoes() {
    World world = withFloor();
    for (int k = 0; k < 20; ++k) world.addBody(Body::box({0.02, 0.2, 0.1}, 1000, {0.2 * k - 2, 0.2, 0}));
    Body& first = world.body(BodyId(1));
    first.angularMomentum = {0, 0, Real(-3) / first.mass.inverseInertia.z};
    return world;
}
std::string dominoVerdict(const World&, const World& now) {
    for (size_t i = 1; i < now.bodies().size(); ++i) {
        const double upright = double(now.bodies()[i].orientation.rotate({0, 1, 0}).y);
        if (upright > 0.5) return text("костяшка %zu не упала", i);
    }
    return "";
}

// --- стена под ударом: шар в 200 кг на скорости 8 м/с --------------------------------------------
// Кладка вперевязку: в чётных рядах шесть кирпичей, в нечётных пять, сдвинутых на полкирпича, —
// так каждый кирпич лежит на двух нижних и ни один не висит центром масс над краем. Кирпичи
// одного ряда уложены вплотную: касаются боками без нагрузки.
World wall() {
    World world = withFloor();
    for (int row = 0; row < 6; ++row)
        for (int k = 0; k < 6 - row % 2; ++k) world.addBody(Body::box({0.1, 0.1, 0.2}, 1000, {0, 0.1 + 0.2 * row, 0.4 * k - 1 + (row % 2 ? 0.2 : 0)}));
    Body ball = Body::sphere(0.25, 3000, {-3, 0.6, 0});
    ball.linearMomentum = {8 / ball.mass.inverseMass, 0, 0};
    world.addBody(ball);
    return world;
}
std::string wallVerdict(const World&, const World& now) {
    for (size_t i = 1; i < now.bodies().size(); ++i)
        if (double(now.bodies()[i].position.y) < 0.05) return text("тело %zu провалилось в пол", i);
    if (double(now.bodies().back().position.x) < 0.5) return "шар не пробил стену";
    return "";
}

// --- пуля: шарик на 500 м/с и пластина толщиной 2 мм ---------------------------------------------
World bullet() {
    World world = withFloor();
    world.addBody(Body::box({0.001, 0.5, 0.5}, 0, {1, 0.5, 0}));
    Body ball = Body::sphere(0.02, 8000, {-1, 0.5, 0});
    ball.linearMomentum = {500 / ball.mass.inverseMass, 0, 0};
    world.addBody(ball);
    return world;
}
std::string bulletVerdict(const World&, const World& now) {
    return double(now.bodies()[2].position.x) < 1 ? "" : text("пуля прошла сквозь пластину: x = %.3f м", double(now.bodies()[2].position.x));
}

// --- маятник Ньютона: пять упругих шаров на подвесах ---------------------------------------------
// Между шарами зазор полмиллиметра: удары идут по цепочке один за другим, и движок, который
// обрывает шаг на каждом ударе, передаёт импульс последнему шару целиком.
World cradle() {
    World world;
    const BodyId beam = world.addBody(Body::box({1, 0.02, 0.02}, 0, {0, 1.5, 0}));
    for (int k = 0; k < 5; ++k) {
        Body ball = Body::sphere(0.1, 7800, {Real(0.2005 * (k - 2)), 0.5, 0});
        ball.restitution = 1, ball.friction = 0;
        const Vector3 pivot{ball.position.x, 1.5, 0};
        if (k == 0) ball.position = pivot + about({0, 0, 1}, -30).rotate({0, -1, 0});
        addBallJoint(world, beam, world.addBody(ball), pivot);
    }
    return world;
}
std::string cradleVerdict(const World&, const World& now) {
    // Через 0,65 с первый шар уже ударил: стоять должны все, кроме последнего.
    for (size_t i = 1; i <= 4; ++i)
        if (double(now.bodies()[i].linearVelocity().length()) > 0.1) return text("шар %zu после удара движется: %.2f м/с", i, double(now.bodies()[i].linearVelocity().length()));
    const double last = double(now.bodies()[5].linearVelocity().length()), expected = std::sqrt(2 * 9.81 * (1 - sinCos(30 * 3.14159265358979323846 / 180).cos));
    return last > 0.8 * expected ? "" : text("последний шар получил %.2f м/с из %.2f", last, expected);
}

// --- цепь из торов: шесть звеньев висят на неподвижном -------------------------------------------
// Соседние звенья повёрнуты на 90° и продеты друг в друга; в покое центры отстоят на 2 (R − r).
// Тор — 32 × 16 граней: при 16 × 8 точка касания звеньев в быстром шаге прыгала с грани на грань, и
// мягкий контакт подкачивал движение — висящая цепь не успокаивалась (0,027 Дж против 0,00003).
const double kRing = 0.2, kTube = 0.04;
World torusChain() {
    World world = withFloor();
    for (int k = 0; k < 7; ++k) {
        Body link = CompoundBodyBuilder::torus(Real(kRing), Real(kTube), k == 0 ? 0 : 2000, {}, 32, 16);
        link.orientation = about(k % 2 ? Vector3{0, 0, 1} : Vector3{1, 0, 0}, 90) * link.orientation;
        link.position = {0, Real(3 - 0.30 * k), 0};
        world.addBody(link);
    }
    return world;
}
std::string chainVerdict(const World&, const World& now) {
    for (int k = 1; k < 7; ++k) {
        const Body& link = now.bodies()[size_t(k + 1)];
        const double expected = 3 - 2 * (kRing - kTube) * k;
        if (std::fabs(double(link.position.y) - expected) > 0.02 * k + 0.01) return text("звено %d висит на высоте %.3f м вместо %.3f", k, double(link.position.y), expected);
        if (speedOf(link) > 0.05) return text("звено %d движется: %.3f м/с", k, speedOf(link));
    }
    return "";
}

// --- ведро в ведре в ведре ------------------------------------------------------------------------
// Одинаковые конические вёдра входят друг в друга, пока стенки не встретятся: дно внутреннего
// останавливается на 6 см выше дна наружного.
World nestedBuckets() {
    World world = withFloor();
    for (int k = 0; k < 3; ++k) world.addBody(CompoundBodyBuilder::bucket(0.15, 0.2, 0.3, 0.01, 800, {0, Real(0.5 * k), 0}));
    return world;
}
std::string bucketsVerdict(const World&, const World& now) {
    const double base = double(now.bodies()[1].position.y);
    for (int k = 1; k < 3; ++k) {
        const Body& bucket = now.bodies()[size_t(k + 1)];
        const double lift = double(bucket.position.y) - base, aside = std::sqrt(double(bucket.position.x) * double(bucket.position.x) + double(bucket.position.z) * double(bucket.position.z));
        if (std::fabs(lift - 0.06 * k) > 0.01) return text("ведро %d стоит на %.3f м выше нижнего вместо %.3f", k + 1, lift, 0.06 * k);
        if (aside > 0.01) return text("ведро %d смещено вбок на %.3f м", k + 1, aside);
        if (speedOf(bucket) > 0.02) return text("ведро %d движется: %.3f м/с", k + 1, speedOf(bucket));
    }
    return "";
}

// --- шары в ведре ---------------------------------------------------------------------------------
// Шары падают ровными столбиками по четыре и стоят друг на друге около 1,6 с, пока шум округления не
// нарушит равновесие; после обвала им нужно ещё около трёх секунд. Поэтому сцена идёт 5 с: к четвёртой
// секунде скорость самого медленного шара лежала у порога проверки (0,04–0,06 м/с), и итог зависел от
// последнего бита решателя, а не от физики (проба с возмущением скорости шара 1–7 мм/с, H88).
World ballsInBucket() {
    World world = withFloor();
    world.addBody(CompoundBodyBuilder::bucket(0.2, 0.25, 0.3, 0.01, 800));
    for (int k = 0; k < 16; ++k) world.addBody(Body::sphere(0.04, 1000, {Real(0.07 * ((k % 4) - 1.5)), Real(0.4 + 0.1 * (k / 4)), Real(0.07 * (((k * 7) % 4) - 1.5))}));
    return world;
}
std::string ballsVerdict(const World&, const World& now) {
    const Body& bucket = now.bodies()[1];
    for (size_t i = 2; i < now.bodies().size(); ++i) {
        const Vector3 p = now.bodies()[i].position - bucket.position;
        if (std::sqrt(double(p.x) * double(p.x) + double(p.z) * double(p.z)) > 0.25 || double(now.bodies()[i].position.y) < 0.04) return text("шар %zu выпал из ведра", i - 1);
        // Покой — по скорости центра. Вращение шара на месте вокруг нормали контакта модель не гасит:
        // у точечного контакта нет момента трения вокруг нормали, и без сопротивления среды шар
        // крутится вечно (1,3 рад/с на замере) — это не движение по ведру.
        const double speed = double(now.bodies()[i].linearVelocity().length());
        if (speed > 0.05) return text("шар %zu движется: %.3f м/с", i - 1, speed);
    }
    return "";
}

// --- куча: тридцать разных тел падают в ящик -----------------------------------------------------
World pile() {
    World world = withFloor();
    for (int side = 0; side < 4; ++side) {
        const Vector3 half = side < 2 ? Vector3{0.05, 0.4, 0.8} : Vector3{0.8, 0.4, 0.05};
        const double shift = side % 2 ? 0.85 : -0.85;
        world.addBody(Body::box(half, 0, side < 2 ? Vector3{Real(shift), 0.4, 0} : Vector3{0, 0.4, Real(shift)}));
    }
    unsigned long long random = 2026;
    const auto uniform = [&] {
        random = random * 6364136223846793005ull + 1442695040888963407ull;
        return double(random >> 11) / double(1ull << 53);
    };
    for (int k = 0; k < 30; ++k) {
        Body body;
        switch (k % 5) {
        case 0: body = Body::box({0.1, 0.07, 0.12}, 1000); break;
        case 1: body = Body::sphere(0.1, 1000); break;
        case 2: body = Body::capsule(0.12, 0.06, 1000); break;
        case 3: body = Body::cylinder(0.08, 0.1, 1000, {}, Real(0.005)); break;
        default: {
            std::vector<Vector3> cloud;
            for (int i = 0; i < 12; ++i) cloud.push_back({Real(0.24 * uniform() - 0.12), Real(0.2 * uniform() - 0.1), Real(0.24 * uniform() - 0.12)});
            body = Body::hull(cloud, 1000);
        }
        }
        body.orientation = (Quaternion{1, Real(uniform() - 0.5), Real(uniform() - 0.5), Real(uniform() - 0.5)} * body.orientation).unit();
        do body.position = {Real(0.9 * uniform() - 0.45), Real(0.5 + 0.25 * k), Real(0.9 * uniform() - 0.45)};
        while (overlapsPlaced(world, body));
        world.addBody(body);
    }
    return world;
}
std::string pileVerdict(const World&, const World& now) {
    for (size_t i = 5; i < now.bodies().size(); ++i) {
        const Vector3 p = now.bodies()[i].position;
        if (double(p.y) < 0.03 || std::fabs(double(p.x)) > 0.8 || std::fabs(double(p.z)) > 0.8) return text("тело %zu вышло из ящика: (%.2f, %.2f, %.2f)", i, double(p.x), double(p.y), double(p.z));
    }
    return "";
}

// --- захват: два пальца на ползунах сжимают кубик и поднимают его ---------------------------------
// Пальцы сводит привод скорости с пределом силы 50 Н; трение 0,5 держит кубик в килограмм силой
// до 2 · 0,5 · 50 = 50 Н. Через секунду каретка с пальцами едет вверх на 0,2 м/с.
World gripper() {
    World world = withFloor();
    const BodyId base = world.addBody(Body::box({0.05, 0.05, 0.05}, 0, {0, 2, 0}));
    const BodyId carriage = world.addBody(Body::box({0.3, 0.03, 0.1}, 500, {0, 0.5, 0}));
    SliderJoint& lift = addSlider(world, base, carriage, {0, 0.5, 0}, {0, 1, 0});
    lift.holdSpeed = true;
    for (int side = -1; side <= 1; side += 2) {
        const BodyId finger = world.addBody(Body::box({0.02, 0.2, 0.05}, 500, {Real(0.15 * side), 0.26, 0}));
        SliderJoint& slide = addSlider(world, carriage, finger, {Real(0.15 * side), 0.5, 0}, {Real(-side), 0, 0});
        slide.holdSpeed = true, slide.driveSpeed = Real(0.1), slide.driveStrength = 50;
    }
    world.addBody(Body::box({0.05, 0.05, 0.05}, 1000, {0, 0.05, 0}));
    return world;
}
void gripperControl(World& world, double time) { world.joint<SliderJoint>(0).driveSpeed = time > 1 ? Real(0.2) : Real(0); }
std::string gripperVerdict(const World& start, const World& now) {
    const Body &cube = now.bodies().back(), &carriage = now.bodies()[2];
    const double lifted = double(cube.position.y - start.bodies().back().position.y), carried = double(carriage.position.y - start.bodies()[2].position.y);
    if (carried < 0.15) return text("каретка поднялась всего на %.3f м", carried);
    if (std::fabs(lifted - carried) > 2e-3) return text("кубик выскальзывает: поднят на %.4f м при подъёме каретки на %.4f", lifted, carried);
    return "";
}

// --- сочленение по траектории: вагонетка на волнистом кольце и коробка на спирали --------------------
// Как пример PathConstraintTest в Jolt. Вагонетку ведёт привод 1,5 м/с по замкнутой кривой, она сохраняет
// ориентацию. Коробка подвешена к нисходящей спирали за середину верхней грани: привода нет, она съезжает
// под тяжестью, свободно вращается и качается, а в конце упирается в предел хода. Вязкое сопротивление вдоль
// пути (1,5 Н·с/м) держит скорость спуска около 2 м/с: без него коробка приходит к упору на 5 м/с, и удар
// раскручивает её до 25 рад/с. Кривые закреплены в полу.
World rail() {
    World world = withFloor();
    const Real c = Real(0.70710678118654752); // cos 45° — без libm
    const Real around[8][2] = {{1, 0}, {c, c}, {0, 1}, {-c, c}, {-1, 0}, {-c, -c}, {0, -1}, {c, -c}};
    std::vector<Vector3> ring, spiral;
    for (int k = 0; k < 8; ++k) ring.push_back({Real(1.2) * around[k][0], k % 2 ? Real(0.95) : Real(0.55), Real(1.2) * around[k][1]});
    for (int k = 0; k <= 24; ++k) { // два витка радиуса 0,8 м вниз с 2,6 до 0,8 м, точка через 30°
        const SinCos turn = sinCos(Real(k) * Real(0.52359877559829887));
        spiral.push_back({Real(3) + Real(0.8) * turn.cos, Real(2.6) - Real(0.075) * Real(k), Real(0.8) * turn.sin});
    }
    const BodyId carriage = world.addBody(Body::box({0.12, 0.06, 0.08}, 500, ring[0]));
    PathJoint& loop = addPathJoint(world, BodyId(0), carriage, ring[0], ring, true);
    loop.lockTurns = true, loop.holdSpeed = true, loop.driveSpeed = Real(1.5);
    Body box = Body::box({0.08, 0.12, 0.05}, 500, spiral[0] - Vector3{0, Real(0.12), 0});
    box.angularDamping = Real(0.5);
    addPathJoint(world, BodyId(0), world.addBody(box), spiral[0], spiral).damping = Real(1.5);
    return world;
}
std::string railVerdict(const World& start, const World& now) {
    const Transform floor = now.bodies()[0].pose();
    for (size_t j = 0; j < 2; ++j) {
        const PathJoint& joint = dynamic_cast<const PathJoint&>(now.joints()[j]);
        const Body& body = now.bodies()[size_t(joint.b)];
        const Vector3 local = floor.localPoint(body.pose().point(joint.anchorB));
        const Real u = joint.path().closest(local);
        const double off = double((local - joint.path().point(u)).length());
        if (off > 1e-3) return text("тело %d ушло с пути на %.2e м", joint.b, off);
        const double along = double(pointVelocity(body, body.pose().point(joint.anchorB)).dot(floor.direction(joint.path().tangent(u))));
        if (j == 0 && std::fabs(along - 1.5) > 0.03) return text("вагонетка идёт по пути со скоростью %.3f м/с вместо 1,5", along);
        const double sunk = double(start.bodies()[size_t(joint.b)].position.y - body.position.y);
        if (j == 1 && sunk < 1.0) return text("коробка съехала по спирали всего на %.2f м высоты", sunk);
    }
    return "";
}

// Равномерное на [0, 1) псевдослучайное число: сцены одинаковы от запуска к запуску.
struct Dice {
    unsigned long long state;
    double roll() {
        state = state * 6364136223846793005ull + 1442695040888963407ull;
        return double(state >> 11) / double(1ull << 53);
    }
    Quaternion turn() { return Quaternion{1, Real(roll() - 0.5), Real(roll() - 0.5), Real(roll() - 0.5)}.unit(); }
    Body hull(double size) {
        std::vector<Vector3> cloud;
        for (int i = 0; i < 12; ++i) cloud.push_back({Real(size * (roll() - 0.5)), Real(size * (roll() - 0.5)), Real(size * (roll() - 0.5))});
        return Body::hull(cloud, 1000);
    }
};

// --- чаша: чайники, лошадки, торы, чашки, ежи и выпуклые тела сыплются в чашу ---------------------
// Чаша — неподвижная сферическая оболочка: внутренний радиус 0,80 м, стенка 5 см, от дна до 80° —
// кромка на высоте 0,70 м, радиусом 0,84 м. Профиль — шесть выпуклых четырёхугольников.
const double kBowlInner = 0.80, kBowlOuter = 0.85, kBowlRim = 0.84;
Body bowlShell() {
    const auto on = [](double radius, double degrees) {
        const double angle = degrees * 3.14159265358979323846 / 180;
        return std::pair<Real, Real>{Real(radius * sinCos(angle).sin), Real(kBowlOuter - radius * sinCos(angle).cos)};
    };
    std::vector<std::vector<std::pair<Real, Real>>> profiles;
    for (int k = 0; k < 5; ++k) profiles.push_back({on(kBowlOuter, 16 * k), on(kBowlOuter, 16 * k + 16), on(kBowlInner, 16 * k + 16), on(kBowlInner, 16 * k)});
    return CompoundBodyBuilder::revolved(profiles, 24, 0);
}
// Чайник: корпус — три усечённых конуса вращения, носик — капсула под 50° к оси, ручка — три бруска,
// шишка крышки — шар. Начало осей — центр дна.
Body teapot() {
    const Real density = 600;
    Body spout = Body::capsule(0.05, 0.016, density, {0.15, 0.09, 0});
    spout.orientation = about({0, 0, 1}, -50);
    return CompoundBodyBuilder::compound({CompoundBodyBuilder::revolved({{{0, 0}, {0.08, 0}, {0.12, 0.06}, {0, 0.06}}, {{0, 0.06}, {0.12, 0.06}, {0.07, 0.13}, {0, 0.13}}, {{0, 0.13}, {0.07, 0.13}, {0.025, 0.15}, {0, 0.15}}}, 10, density),
                                          spout, Body::box({0.035, 0.01, 0.01}, density, {-0.14, 0.11, 0}), Body::box({0.01, 0.04, 0.01}, density, {-0.17, 0.08, 0}),
                                          Body::box({0.035, 0.01, 0.01}, density, {-0.14, 0.05, 0}), Body::sphere(0.02, density, {0, 0.165, 0})});
}
// Лошадка: туловище, четыре ноги, шея, голова и хвост — бруски.
Body horse() {
    const Real density = 700;
    std::vector<Body> pieces{Body::box({0.11, 0.04, 0.035}, density, {0, 0.16, 0})};
    for (int k = 0; k < 4; ++k) pieces.push_back(Body::box({0.014, 0.06, 0.014}, density, {Real(k < 2 ? 0.085 : -0.085), 0.06, Real(k % 2 ? 0.022 : -0.022)}));
    Body neck = Body::box({0.02, 0.055, 0.02}, density, {0.115, 0.225, 0}), head = Body::box({0.045, 0.022, 0.02}, density, {0.165, 0.275, 0}), tail = Body::box({0.012, 0.04, 0.012}, density, {-0.12, 0.15, 0});
    neck.orientation = about({0, 0, 1}, -30), head.orientation = about({0, 0, 1}, -20), tail.orientation = about({0, 0, 1}, -25);
    pieces.insert(pieces.end(), {neck, head, tail});
    return CompoundBodyBuilder::compound(pieces);
}
// Ёж: три бруска крест-накрест по осям.
Body jack() { return CompoundBodyBuilder::compound({Body::box({0.09, 0.018, 0.018}, 800), Body::box({0.018, 0.09, 0.018}, 800), Body::box({0.018, 0.018, 0.09}, 800)}); }
World bowl() {
    World world = withFloor();
    world.addBody(bowlShell());
    Dice dice{2027};
    for (int k = 0; k < 24; ++k) {
        Body body;
        switch (k % 6) {
        case 0: body = teapot(); break;
        case 1: body = horse(); break;
        case 2: body = CompoundBodyBuilder::torus(0.09, 0.022, 900); break;
        case 3: body = CompoundBodyBuilder::bucket(0.045, 0.06, 0.09, 0.007, 900); break;
        case 4: body = jack(); break;
        default: body = k % 12 == 5 ? dice.hull(0.2) : Body::cylinder(0.06, 0.07, 1000, {}, Real(0.005));
        }
        body.orientation = (dice.turn() * body.orientation).unit();
        body.position = {Real(0.3 * sinCos(2.4 * k).cos), Real(0.9 + 0.25 * k), Real(0.3 * sinCos(2.4 * k).sin)};
        world.addBody(body);
    }
    return world;
}
std::string bowlVerdict(const World&, const World& now) {
    for (size_t i = 2; i < now.bodies().size(); ++i) {
        const Body& body = now.bodies()[i];
        if (std::sqrt(double(body.position.x) * double(body.position.x) + double(body.position.z) * double(body.position.z)) > kBowlRim || double(body.position.y) < 0.02) return text("тело %zu вне чаши: (%.2f, %.2f, %.2f)", i - 1, double(body.position.x), double(body.position.y), double(body.position.z));
        if (speedOf(body) > 0.05) return text("тело %zu движется: %.3f м/с", i - 1, speedOf(body));
    }
    return "";
}

// --- холмы: тела падают на рельеф, заданный картой высот -------------------------------------------
// Высота h(x, z) = 0,2 + 0,03 (x² + z²) + 0,12 sin(1,7 x) cos(1,3 z) на сетке 33 × 33 узла с шагом
// 0,25 м: 8 × 8 м, 2048 треугольников (heightfieldMesh → CompoundBodyBuilder::ground). Плоского пола нет.
const int kHillSide = 33;
const double kHillCell = 0.25;
double hillHeight(double x, double z) { return 0.2 + 0.03 * (x * x + z * z) + 0.12 * sinCos(1.7 * x).sin * sinCos(1.3 * z).cos; }
World hills() {
    World world;
    std::vector<Real> heights;
    for (int k = 0; k < kHillSide; ++k)
        for (int i = 0; i < kHillSide; ++i) heights.push_back(Real(hillHeight((i - (kHillSide - 1) / 2) * kHillCell, (k - (kHillSide - 1) / 2) * kHillCell)));
    world.addBody(CompoundBodyBuilder::ground(heightfieldMesh(heights, kHillSide, kHillSide, Real(kHillCell))));
    Dice dice{2028};
    for (int k = 0; k < 20; ++k) {
        Body body;
        switch (k % 5) {
        case 0: body = Body::box({0.12, 0.08, 0.15}, 1000); break;
        case 1: body = Body::sphere(0.11, 1000); break;
        case 2: body = Body::capsule(0.13, 0.07, 1000); break;
        case 3: body = Body::cylinder(0.09, 0.11, 1000, {}, Real(0.005)); break;
        default: body = dice.hull(0.28);
        }
        const double x = 3 * dice.roll() - 1.5, z = 3 * dice.roll() - 1.5;
        body.orientation = (dice.turn() * body.orientation).unit();
        body.position = {Real(x), Real(hillHeight(x, z) + 0.5 + 0.15 * k), Real(z)};
        world.addBody(body);
    }
    return world;
}
std::string hillsVerdict(const World&, const World& now) {
    for (size_t i = 1; i < now.bodies().size(); ++i) {
        const Vector3 p = now.bodies()[i].position;
        if (std::fabs(double(p.x)) > 4 || std::fabs(double(p.z)) > 4) return text("тело %zu ушло за край рельефа: (%.2f, %.2f, %.2f)", i, double(p.x), double(p.y), double(p.z));
        if (double(p.y) < hillHeight(double(p.x), double(p.z))) return text("тело %zu под рельефом: высота %.3f м при рельефе %.3f", i, double(p.y), hillHeight(double(p.x), double(p.z)));
    }
    return "";
}

// --- сторожа сохранения ----------------------------------------------------------------------------
// Полная механическая энергия: движения плюс высоты в поле тяжести мира.
double mechanicalEnergy(const World& world) {
    double energy = 0;
    for (const Body& body : world.bodies())
        if (!body.isStatic()) energy += double(kineticEnergyOf(body)) - double(world.gravity.dot(body.position)) / double(body.mass.inverseMass);
    return energy;
}

// Раскачанная цепь: та же цепь из торов, нижнему звену дана скорость 4 м/с вбок (101 Дж). Трение и
// удары энергию только отбирают, поэтому в конце её не может быть больше, чем в начале. Замороженный
// на шаг мира контакт быстрого шага её подкачивал: 54 Дж на 10-й секунде и 141 на 40-й.
World swingingChain() {
    World world = torusChain();
    Body& last = world.body(BodyId(int(world.bodies().size()) - 1));
    last.linearMomentum = {4 / last.mass.inverseMass, 0, 0};
    return world;
}
std::string swingVerdict(const World& start, const World& now) {
    const double before = mechanicalEnergy(start), after = mechanicalEnergy(now), put = double(kineticEnergyOf(start.bodies().back()));
    if (after > before + 1e-3 * put) return text("энергия выросла на %.1f Дж при вложенных %.1f", after - before, put);
    return "";
}

// Куча без тяжести и без неподвижных тел: двадцать тел летят к центру и сталкиваются. Внешних сил нет,
// поэтому импульс и момент импульса системы обязаны остаться прежними; допуск — 10⁻⁹ от суммы модулей
// импульсов тел (момента — на метр плеча).
World weightlessPile() {
    World world;
    world.gravity = {};
    Dice dice{2029};
    for (int k = 0; k < 20; ++k) {
        Body body = k % 4 == 0 ? Body::sphere(0.1, 1000) : k % 4 == 1 ? Body::capsule(0.12, 0.06, 1000) : k % 4 == 2 ? dice.hull(0.25) : Body::box({0.1, 0.07, 0.12}, 1000);
        body.orientation = (dice.turn() * body.orientation).unit();
        do body.position = {Real(1.6 * dice.roll() - 0.8), Real(1.6 * dice.roll() - 0.8), Real(1.6 * dice.roll() - 0.8)};
        while (overlapsPlaced(world, body));
        const Vector3 from = body.position;
        body.linearMomentum = (Real(-1.5 - dice.roll()) / body.mass.inverseMass) * from;
        body.angularMomentum = {Real(0.05 * (dice.roll() - 0.5)), Real(0.05 * (dice.roll() - 0.5)), Real(0.05 * (dice.roll() - 0.5))};
        world.addBody(body);
    }
    return world;
}
struct Momenta {
    Vector3 linear, angular;
    double scale = 0;
};
Momenta momentaOf(const World& world) {
    Momenta m;
    for (const Body& body : world.bodies()) {
        m.linear += body.linearMomentum, m.angular += body.position.cross(body.linearMomentum) + body.angularMomentum;
        m.scale += double(body.linearMomentum.length());
    }
    return m;
}
std::string driftVerdict(const World& start, const World& now) {
    const Momenta before = momentaOf(start), after = momentaOf(now);
    const double linear = double((after.linear - before.linear).length()) / before.scale, angular = double((after.angular - before.angular).length()) / before.scale;
    if (linear > 1e-9) return text("импульс системы ушёл на %.2e от суммы импульсов тел", linear);
    if (angular > 1e-9) return text("момент импульса системы ушёл на %.2e (на метр плеча)", angular);
    return "";
}

} // namespace

const std::vector<StandardScene>& standardScenes() {
    static const std::vector<StandardScene> all = {
        {"stack", "стопка из десяти кубов", stack, 5, 5, stackVerdict, nullptr, {0, 2, 0}, 8},
        {"pyramid", "пирамида из 36 кубов", pyramid, 5, 5, pyramidVerdict, nullptr, {0, 1.5, 0}, 8},
        {"heavy", "куб в тонну на пяти кубах по килограмму", heavyOnLight, 5, 5, heavyVerdict, nullptr, {0, 1.2, 0}, 6},
        {"arch", "арка из девяти клиньев", arch, 5, 5, archVerdict, nullptr, {0, 0.6, 0}, 5},
        {"slope", "бруски с разным трением на склоне", slope, 1, 1, slopeVerdict, nullptr, {-2, -1, 0}, 8},
        {"dominoes", "двадцать костяшек домино", dominoes, 6, 6, dominoVerdict, nullptr, {0, 0.3, 0}, 6},
        {"wall", "стена из 33 кирпичей под ударом шара", wall, 4, 4, wallVerdict, nullptr, {0, 0.6, 0}, 7},
        {"bullet", "пуля 500 м/с и пластина 2 мм", bullet, 1, 1, bulletVerdict, nullptr, {0, 0.5, 0}, 4},
        {"cradle", "маятник Ньютона", cradle, 4, 0.65, cradleVerdict, nullptr, {0, 0.9, 0}, 4},
        {"chain", "цепь из шести торов", torusChain, 5, 5, chainVerdict, nullptr, {0, 2, 0}, 5},
        {"buckets", "ведро в ведре в ведре", nestedBuckets, 4, 4, bucketsVerdict, nullptr, {0, 0.4, 0}, 3},
        {"balls", "шестнадцать шаров в ведре", ballsInBucket, 5, 5, ballsVerdict, nullptr, {0, 0.3, 0}, 2.5},
        {"pile", "куча из тридцати тел в ящике", pile, 6, 6, pileVerdict, nullptr, {0, 0.5, 0}, 5},
        {"gripper", "захват поднимает кубик", gripper, 2.5, 2.5, gripperVerdict, gripperControl, {0, 0.4, 0}, 3},
        {"bowl", "чаша: чайники, лошадки, торы, чашки, ежи", bowl, 10, 10, bowlVerdict, nullptr, {0, 0.5, 0}, 4},
        {"hills", "холмы по карте высот", hills, 8, 8, hillsVerdict, nullptr, {0, 0.6, 0}, 9},
        {"swing", "раскачанная цепь: энергия не растёт", swingingChain, 40, 40, swingVerdict, nullptr, {0, 2, 0}, 6},
        {"drift", "куча без тяжести: импульс сохраняется", weightlessPile, 5, 5, driftVerdict, nullptr, {0, 0, 0}, 6},
        {"rail", "вагонетка на кольцевом пути и коробка на спирали", rail, 7, 7, railVerdict, nullptr, {1.5, 1.2, 0}, 6},
    };
    return all;
}
