// Прибор: быстрые столкновения для всех пар фигур. Снаряд каждой формы летит в тонкую неподвижную
// мишень каждой формы на скоростях от 1 м/с до 10 км/с, со случайным поворотом, вращением и
// смещением от центра. Прогон обычным шагом 1/60 с — за него снаряд пролетает до 167 м при
// толщине мишени 1 см — сравнивается с эталоном: тем же движком, но с шагом, за который снаряд
// сдвигается на 2 мм и пролететь мишень не может. Движок обязан дать тот же исход — удар или
// промах — и не допустить перекрытия тел. Удар — это изменение скорости снаряда: после скользящего
// удара о круглую мишень снаряд законно летит дальше, поэтому «оказался за мишенью» — не признак.

#include <kp/collision/ContactPair.h>
#include <kp/collision/narrowphase/Gjk.h>
#include <kp/collision/narrowphase/Sat.h>
#include <kp/world/World.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

using namespace kp;

namespace {

double uniform(std::uint64_t& state) { // детерминированный генератор, [0, 1)
    state = state * 6364136223846793005ull + 1442695040888963407ull;
    return double(state >> 11) / double(1ull << 53);
}

// Окружность из n точек радиуса r в плоскости x = offset.
void ring(std::vector<Vector3>& points, int n, Real r, Real offset) {
    for (int k = 0; k < n; ++k) points.push_back({offset, Real(r * std::cos(6.283185307179586 * k / n)), Real(r * std::sin(6.283185307179586 * k / n))});
}

const char* kProjectiles[] = {"шар", "капсула", "цилиндр", "скругл. цилиндр", "куб", "оболочка", "стержень"};
const char* kTargets[] = {"пластина", "шар", "брус-капсула", "диск-цилиндр", "плита-оболочка"};

Body projectile(int kind, std::uint64_t& random) {
    std::vector<Vector3> points;
    switch (kind) {
    case 0: return Body::sphere(0.1, 1000);
    case 1: return Body::capsule(0.2, 0.05, 1000);
    case 2: return Body::cylinder(0.1, 0.1, 1000);
    case 3: return Body::cylinder(0.1, 0.1, 1000, {}, 0.01);
    case 4: return Body::box({0.1, 0.1, 0.1}, 1000);
    case 5:
        for (int k = 0; k < 12; ++k) points.push_back({0.2 * uniform(random) - 0.1, 0.2 * uniform(random) - 0.1, 0.2 * uniform(random) - 0.1});
        return Body::hull(points, 1000);
    default:
        for (int i = 0; i < 8; ++i) points.push_back({i & 1 ? 0.02 : -0.02, i & 2 ? 0.3 : -0.3, i & 4 ? 0.02 : -0.02});
        return Body::hull(points, 1000);
    }
}

// Мишени неподвижны, стоят в начале координат поперёк оси X; толщина вдоль X — сантиметр, у
// бруса-капсулы шесть, у шара — его диаметр.
Body target(int kind) {
    std::vector<Vector3> points;
    Body body;
    switch (kind) {
    case 0: return Body::box({0.005, 1, 1}, 0);
    case 1: return Body::sphere(0.3, 0);
    case 2:
        body = Body::capsule(1, 0.03, 0);
        body.orientation = Quaternion{1, 1, 0, 0}.unit(); // ось — вдоль Z
        return body;
    case 3:
        body = Body::cylinder(0.005, 1, 0);
        body.orientation = Quaternion{1, 0, 0, -1}.unit(); // ось — вдоль X
        return body;
    default:
        ring(points, 24, 1, -0.005), ring(points, 24, 1, 0.005);
        return Body::hull(points, 0);
    }
}

// Глубина перекрытия двух тел; ноль — не перекрыты. У перекрывшихся ядер без многогранной
// топологии глубина неизвестна — возвращается −1.
double depthOf(const Body& a, const Body& b) {
    const auto* hullA = dynamic_cast<const ConvexHull*>(a.shape.get());
    const auto* hullB = dynamic_cast<const ConvexHull*>(b.shape.get());
    if (hullA && hullB) return std::fmax(0.0, -findSeparatingAxis(*hullA, a.pose(), *hullB, b.pose()).separation);
    const ClosestPoints gap = closestPoints(*a.shape, a.pose(), *b.shape, b.pose());
    return gap.coresOverlap ? -1 : std::fmax(0.0, -gap.distance);
}

struct DepthWatch final : StepObserver {
    double deepest = 0;
    bool coresOverlapped = false;
    void after(StepStage stage, const World& world) override {
        if (stage != StepStage::Motion) return;
        const double depth = depthOf(world.bodies()[0], world.bodies()[1]);
        coresOverlapped = coresOverlapped || depth < 0;
        deepest = std::fmax(deepest, depth);
    }
};

struct Outcome {
    bool hit; // скорость снаряда изменилась
    Vector3 velocity;
    double deepest;
    bool coresOverlapped;
    int segments; // наибольшее число отрезков в одном шаге
};

Outcome fly(World world, Real dt, int steps) {
    DepthWatch watch;
    world.observer = &watch;
    const Vector3 before = world.bodies()[1].linearVelocity();
    int segments = 0;
    for (int step = 0; step < steps; ++step) world.step(dt), segments = std::max(segments, world.report().segments);
    const Vector3 after = world.bodies()[1].linearVelocity();
    return {(after - before).length() > 1e-9 * before.length(), after, watch.deepest, watch.coresOverlapped, segments};
}

// --- Столкновения от одного вращения ------------------------------------------------------------
// Тело висит на месте и только вращается; в круге, который оно заметает, стоит тонкая неподвижная
// преграда. Центр тела не движется, поэтому столкновение — целиком дело поворота: при 10 000 рад/с
// тело делает 26 оборотов за шаг 1/60 с, а конец стержня идёт со скоростью 3 км/с. Эталон — тот же
// движок с шагом, за который тело поворачивается на 0,02 рад.

const char* kSpinners[] = {"стержень", "пластина", "куб", "капсула", "цилиндр"};
const char* kObstacles[] = {"столбик-капсула", "шарик", "тонкая пластина"};

// Тело, вытянутое вдоль оси X. reach — расстояние от центра до самой далёкой его точки поперёк оси
// вращения Z, width — его полуширина вдоль Y: преграда ставится сбоку, за пределами тела.
Body spinner(int kind, Real& reach, Real& width) {
    std::vector<Vector3> points;
    Body body;
    switch (kind) {
    case 0:
        for (int i = 0; i < 8; ++i) points.push_back({i & 1 ? 0.3 : -0.3, i & 2 ? 0.02 : -0.02, i & 4 ? 0.02 : -0.02});
        reach = 0.3, width = 0.02;
        return Body::hull(points, 1000);
    case 1:
        for (int i = 0; i < 8; ++i) points.push_back({i & 1 ? 0.3 : -0.3, i & 2 ? 0.2 : -0.2, i & 4 ? 0.01 : -0.01});
        reach = 0.3, width = 0.2;
        return Body::hull(points, 1000);
    case 2: reach = Real(0.2828), width = 0.2; return Body::box({0.2, 0.2, 0.2}, 1000);
    case 3:
        body = Body::capsule(0.3, 0.02, 1000);
        body.orientation = Quaternion{1, 0, 0, 1}.unit(); // ось капсулы — вдоль X
        reach = 0.32, width = 0.02;
        return body;
    default:
        body = Body::cylinder(0.3, 0.03, 1000);
        body.orientation = Quaternion{1, 0, 0, 1}.unit();
        reach = 0.3, width = 0.03;
        return body;
    }
}

// Преграда на расстоянии distance от оси вращения, под углом azimuth.
Body obstacle(int kind, Real distance, double azimuth) {
    const Vector3 at{Real(distance * std::cos(azimuth)), Real(distance * std::sin(azimuth)), 0};
    Body body;
    switch (kind) {
    case 0: // столбик диаметром 1 см вдоль оси вращения
        body = Body::capsule(0.5, 0.005, 0, at);
        body.orientation = Quaternion{1, 1, 0, 0}.unit();
        return body;
    case 1: return Body::sphere(0.01, 0, at);
    default: // пластина толщиной 4 мм, стоящая ребром к оси: её плоскость проходит через ось
        body = Body::box({0.01, 0.002, 0.3}, 0, at);
        body.orientation = Quaternion{Real(std::cos(azimuth / 2)), 0, 0, Real(std::sin(azimuth / 2))}.unit();
        return body;
    }
}

Outcome spin(World world, Real dt, int steps) {
    DepthWatch watch;
    world.observer = &watch;
    const Vector3 before = world.bodies()[1].angularMomentum;
    int segments = 0;
    for (int step = 0; step < steps; ++step) world.step(dt), segments = std::max(segments, world.report().segments);
    const Vector3 after = world.bodies()[1].angularMomentum;
    return {(after - before).length() > 1e-9 * before.length(), world.bodies()[1].linearVelocity(), watch.deepest, watch.coresOverlapped, segments};
}

int spinTable(int trials, double fastest) {
    std::uint64_t random = 2027;
    int total = 0, missed = 0, falseHits = 0, mostSegments = 0;
    double deepestEver = 0;
    std::printf("%-10s %-16s %12s %6s %7s %10s %8s %15s %10s\n", "тело", "преграда", "вращение", "проб", "ударов", "пропущено", "ложных", "перекрытие, мм", "отрезков");
    for (int s = 0; s < 5; ++s) {
        for (int o = 0; o < 3; ++o) {
            for (double speed : {10.0, 100.0, 1000.0, 10000.0}) {
                if (speed > fastest) continue;
                int hits = 0, cellMissed = 0, cellFalse = 0, cellSegments = 0;
                double cellDeepest = 0;
                for (int trial = 0; trial < trials; ++trial) {
                    World world;
                    world.gravity = {};
                    Real reach = 0, width = 0;
                    Body body = spinner(s, reach, width);
                    // Преграда стоит сбоку от тела — за его шириной, но внутри круга, который
                    // заметают его концы: в начале тела не касаются, а через четверть оборота
                    // обязаны столкнуться.
                    const double azimuth = 1.5707963267948966 + 0.3 * (uniform(random) - 0.5);
                    const Real nearest = width + Real(0.03), farthest = std::max(nearest, reach - Real(0.02));
                    world.addBody(obstacle(o, nearest + Real(uniform(random)) * (farthest - nearest), azimuth));
                    const Vector3 inverse = body.mass.inverseInertia;
                    const Vector3 local = body.orientation.rotateInverse({0, 0, Real(speed)});
                    body.angularMomentum = body.orientation.rotate({local.x / inverse.x, local.y / inverse.y, local.z / inverse.z});
                    world.addBody(body);

                    const double duration = 1.5 * 6.283185307179586 / speed; // полтора оборота
                    const int coarseSteps = std::max(1, int(std::ceil(duration * 60)));
                    const Outcome coarse = spin(world, Real(1) / 60, coarseSteps);
                    const int fine = int(std::ceil(duration * speed / 0.02));
                    const Outcome reference = spin(world, Real(duration / fine), fine);

                    hits += reference.hit;
                    cellMissed += !coarse.hit && reference.hit;
                    cellFalse += coarse.hit && !reference.hit;
                    cellSegments = std::max(cellSegments, coarse.segments);
                    cellDeepest = std::fmax(cellDeepest, coarse.deepest);
                }
                total += trials, missed += cellMissed, falseHits += cellFalse, mostSegments = std::max(mostSegments, cellSegments);
                deepestEver = std::fmax(deepestEver, cellDeepest);
                std::printf("%-10s %-16s %7g рад/с %5d %7d %10d %8d %15.4f %10d\n", kSpinners[s], kObstacles[o], speed, trials, hits, cellMissed, cellFalse, 1e3 * cellDeepest,
                            cellSegments);
                std::fflush(stdout);
            }
        }
    }
    std::printf("\nвсего проб %d: пропущенных столкновений %d, ложных %d\n", total, missed, falseHits);
    std::printf("наибольшее перекрытие %.4f мм; отрезков в одном шаге не больше %d\n", 1e3 * deepestEver, mostSegments);
    return missed != 0;
}

} // namespace

int main(int argc, char** argv) {
    const int trials = argc > 1 ? std::atoi(argv[1]) : 10;
    const double spinLimit = argc > 2 ? std::atof(argv[2]) : 10; // наибольшая начальная угловая скорость снаряда, рад/с
    // Третий аргумент spin — вместо летящих снарядов проверяются столкновения от одного вращения.
    // Четвёртый — наибольшая скорость вращения в этой проверке, рад/с: на 10 000 рад/с тело за шаг
    // делает 26 оборотов и бьётся о преграду сотни раз, такой случай считается минутами.
    if (argc > 3 && std::string(argv[3]) == "spin") return spinTable(trials, argc > 4 ? std::atof(argv[4]) : 10000);
    // Для летящих снарядов третий аргумент — наибольшая скорость, м/с: после удара на 10 км/с тело
    // вращается так быстро, что одна проба считается минутами.
    const double fastest = argc > 3 ? std::atof(argv[3]) : 10000;
    std::uint64_t random = 2026;
    int total = 0, tunnels = 0, falseHits = 0, overlaps = 0, mostSegments = 0;
    double deepestEver = 0, worstVelocity = 0;
    std::printf("%-16s %-15s %10s %6s %6s %9s %8s %15s %12s %10s\n", "снаряд", "мишень", "скорость", "проб", "ударов", "пролётов", "ложных", "перекрытие, мм", "скорость, %", "отрезков");
    for (int p = 0; p < 7; ++p) {
        for (int t = 0; t < 5; ++t) {
            for (double speed : {1.0, 100.0, 10000.0}) {
                if (speed > fastest) continue;
                int bounces = 0, cellTunnels = 0, cellFalse = 0, cellSegments = 0;
                double cellDeepest = 0, cellVelocity = 0;
                for (int trial = 0; trial < trials; ++trial) {
                    World world;
                    world.gravity = {};
                    world.addBody(target(t));
                    Body body = projectile(p, random);
                    body.position = {-1.5, 0.4 * uniform(random) - 0.2, 0.4 * uniform(random) - 0.2};
                    body.orientation = Quaternion{2 * uniform(random) - 1, 2 * uniform(random) - 1, 2 * uniform(random) - 1, 2 * uniform(random) - 1}.unit();
                    body.linearMomentum = {speed / body.mass.inverseMass, 0, 0};
                    // Вращение вокруг случайной оси: L = R I Rᵀ ω.
                    const Vector3 spin = body.orientation.rotateInverse(spinLimit * Vector3{2 * uniform(random) - 1, 2 * uniform(random) - 1, 2 * uniform(random) - 1});
                    const Vector3 inverse = body.mass.inverseInertia;
                    body.angularMomentum = body.orientation.rotate({spin.x / inverse.x, spin.y / inverse.y, spin.z / inverse.z});
                    world.addBody(body);

                    // Обычный прогон — шагом 1/60 с. Эталон идёт столько же времени, пока это не
                    // слишком долго; на 10 км/с — только до удара и чуть дальше, и скорости после
                    // удара там не сравниваются: за остаток шага снаряд может удариться ещё раз.
                    const int coarseSteps = std::max(3, int(std::ceil(2.5 * 1.5 / speed * 60)));
                    const bool sameDuration = speed <= 100;
                    const double duration = sameDuration ? coarseSteps / 60.0 : 2.5 * 1.5 / speed;
                    const Outcome coarse = fly(world, Real(1) / 60, coarseSteps);
                    const int fine = int(std::ceil(duration * speed / 0.002));
                    const Outcome reference = fly(world, Real(duration / fine), fine);

                    bounces += reference.hit;
                    cellTunnels += !coarse.hit && reference.hit;
                    cellFalse += coarse.hit && !reference.hit;
                    overlaps += coarse.coresOverlapped;
                    cellSegments = std::max(cellSegments, coarse.segments);
                    cellDeepest = std::fmax(cellDeepest, coarse.deepest);
                    if (sameDuration && coarse.hit && reference.hit)
                        cellVelocity = std::fmax(cellVelocity, (coarse.velocity - reference.velocity).length() / speed);
                }
                total += trials, tunnels += cellTunnels, falseHits += cellFalse, mostSegments = std::max(mostSegments, cellSegments);
                deepestEver = std::fmax(deepestEver, cellDeepest), worstVelocity = std::fmax(worstVelocity, cellVelocity);
                std::printf("%-16s %-15s %8g м/с %5d %6d %9d %8d %15.4f %12.4f %10d\n", kProjectiles[p], kTargets[t], speed, trials, bounces, cellTunnels, cellFalse,
                            1e3 * cellDeepest, 100 * cellVelocity, cellSegments);
                std::fflush(stdout);
            }
        }
    }
    std::printf("\nвсего проб %d: пролётов сквозь мишень %d, ложных столкновений %d, перекрытий ядер %d\n", total, tunnels, falseHits, overlaps);
    std::printf("наибольшее перекрытие %.4f мм; отрезков в одном шаге не больше %d\n", 1e3 * deepestEver, mostSegments);
    std::printf("наибольшее расхождение скорости после удара с эталоном (до 100 м/с): %.4f %%\n", 100 * worstVelocity);
    return tunnels != 0;
}
