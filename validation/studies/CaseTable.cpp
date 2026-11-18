// Прибор: полная таблица случаев контакта двух тел. Оси таблицы независимы:
//   чем касается тело A  — 15 вариантов: каждая фигура каждым своим элементом;
//   чего касается у B    — те же 15;
//   отношение размеров   — A впятеро меньше B, равны, впятеро больше;
//   режим                — покой, склон, волчок и удары на четырёх скоростях, по неподвижному и
//                          по свободному телу.
// Клетка — одно сочетание значений всех осей, перебираются все. Внутри клетки случайны поворот
// вокруг нормали, смещение и наклон; первая проба — точное совмещение элементов: грань строго
// на грани, вершина строго в вершину. Именно вырожденные положения ломают геометрию.
//
// Мерки у всех клеток одни и не требуют знать ответ заранее:
//   перекрытие     — тела не входят друг в друга;
//   энергия        — при нулевом восстановлении механическая энергия не растёт: мерка — наибольший
//                    подъём над самым низким уровнем, до которого она уже опускалась. Сумма всех
//                    приращений не годится: у свободно вращающегося несимметричного тела энергия
//                    колеблется в пределах 10⁻⁴ и не растёт, а сумма набирала вдвое больше порога
//                    без единого контакта. Из подъёма вычитается работа выталкивания из допуска
//                    перекрытия, m g · 2δ: движок вправе поднять тело на глубину, на которую
//                    вправе его впустить;
//   импульс, момент — пара свободных тел без тяжести сохраняет импульс и момент импульса;
//   пролёт         — тело, пущенное элементом в элемент, обязано удариться; если удара нет,
//                    решает эталон — тот же движок с шагом, за который тело сдвигается на 2 мм;
//   покой          — тело, поставленное устойчиво, через секунду стоит там же;
//   числа          — в состоянии нет бесконечностей и «не чисел»;
//   конец шага     — в шаге не больше 10 000 отрезков, и движок не остановил его сам.

#include <kp/collision/ContactPair.h>
#include <kp/collision/narrowphase/Gjk.h>
#include <kp/world/World.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

using namespace kp;

namespace {

double uniform(std::uint64_t& state) { // детерминированный генератор, [0, 1)
    state = state * 6364136223846793005ull + 1442695040888963407ull;
    return double(state >> 11) / double(1ull << 53);
}

// --- Оси таблицы ---------------------------------------------------------------------------------

enum Shape { Ball, CapsuleShape, SharpCylinder, RoundedCylinder, Brick, Hull };
const char* kShapes[] = {"шар", "капсула", "цилиндр", "скругл. цилиндр", "брусок", "оболочка"};

// Элемент поверхности, которым тело касается: грань — плоский участок, ребро — прямое ребро
// многогранника или кромка торца цилиндра, вершина, бок — образующая цилиндра или капсулы,
// выпуклость — шар и конец капсулы.
enum Element { Face, Edge, Vertex, Side, Bulge };
const char* kElements[] = {"грань", "ребро", "вершина", "бок", "выпуклость"};

struct Variant {
    Shape shape;
    Element element;
};
const Variant kVariants[] = {{Ball, Bulge},          {CapsuleShape, Bulge},   {CapsuleShape, Side},  {SharpCylinder, Face}, {SharpCylinder, Edge},
                             {SharpCylinder, Side},  {RoundedCylinder, Face}, {RoundedCylinder, Edge}, {RoundedCylinder, Side}, {Brick, Face},
                             {Brick, Edge},          {Brick, Vertex},         {Hull, Face},          {Hull, Edge},          {Hull, Vertex}};
const int kVariantCount = sizeof kVariants / sizeof *kVariants;

const Real kSizes[] = {Real(0.2), 1, 5};
const char* kSizeNames[] = {"A впятеро меньше", "размеры равны", "A впятеро больше"};

struct Regime {
    const char* name;
    Vector3 gravity;
    Real clearance; // зазор между телами в начале, м
    Real speed;     // скорость A навстречу B, м/с
    Real spin;      // угловая скорость A, рад/с
    bool spinAboutNormal, freeTarget;
    int steps; // шагов по 1/60 с
};
const Real kTouching = Real(0.5) * kContactDistance;
const Vector3 kDown{0, Real(-9.81), 0}, kSlope{Real(5.6268), Real(-8.0359), 0}; // склон 35°: tg 35° = 0,70 > трения 0,5
const Regime kRegimes[] = {
    {"покой", kDown, kTouching, 0, 0, false, false, 60},
    {"склон 35°", kSlope, kTouching, 0, 0, false, false, 60},
    {"волчок 20 рад/с", kDown, kTouching, 0, 20, true, false, 60},
    {"удар 1 м/с", {}, Real(0.5), 1, 0, false, false, 60},
    {"удар 100 м/с", {}, Real(0.5), 100, 0, false, false, 5},
    {"удар 10 км/с", {}, Real(0.5), 10000, 0, false, false, 5},
    {"удар 1 м/с с вращением", {}, Real(0.5), 1, 30, false, false, 60},
    {"удар 1 м/с, B свободно", {}, Real(0.5), 1, 0, false, true, 60},
    {"удар 100 м/с, B свободно", {}, Real(0.5), 100, 0, false, true, 5},
    {"удар 10 км/с, B свободно", {}, Real(0.5), 10000, 0, false, true, 5},
    {"удар с вращением, B свободно", {}, Real(0.5), 1, 30, false, true, 60},
};
const int kRegimeCount = sizeof kRegimes / sizeof *kRegimes;

// --- Тела и их элементы --------------------------------------------------------------------------

Body make(Shape shape, Real s, Real density, std::uint64_t& random) {
    std::vector<Vector3> points;
    switch (shape) {
    case Ball: return Body::sphere(Real(0.25) * s, density);
    case CapsuleShape: return Body::capsule(Real(0.25) * s, Real(0.1) * s, density);
    case SharpCylinder: return Body::cylinder(Real(0.2) * s, Real(0.25) * s, density);
    case RoundedCylinder: return Body::cylinder(Real(0.2) * s, Real(0.25) * s, density, {}, Real(0.03) * s);
    case Brick: return Body::box({Real(0.25) * s, Real(0.15) * s, Real(0.2) * s}, density);
    default:
        for (int k = 0; k < 12; ++k) points.push_back({s * Real(0.5 * uniform(random) - 0.25), s * Real(0.4 * uniform(random) - 0.2), s * Real(0.3 * uniform(random) - 0.15)});
        return Body::hull(points, density);
    }
}

Vector3 unit(Vector3 v) { return (1 / v.length()) * v; }

// Точка элемента и направление наружу в ней, в осях тела.
struct Spot {
    Vector3 point, outward;
};
Spot spotOf(const Body& body, Element element, std::uint64_t& random) {
    if (const auto* hull = dynamic_cast<const ConvexHull*>(body.shape.get())) {
        const auto pick = [&](size_t count) { return size_t(uniform(random) * double(count)); };
        if (element == Face) {
            const ConvexHull::Face& face = hull->faces()[pick(hull->faces().size())];
            Vector3 centre;
            for (int v : face.loop) centre += (Real(1) / Real(face.loop.size())) * hull->vertices()[size_t(v)];
            return {centre, face.normal};
        }
        if (element == Edge) {
            const ConvexHull::Edge& edge = hull->edges()[pick(hull->edges().size())];
            return {Real(0.5) * (hull->vertices()[size_t(edge.tail)] + hull->vertices()[size_t(edge.head)]),
                    unit(hull->faces()[size_t(edge.left)].normal + hull->faces()[size_t(edge.right)].normal)};
        }
        const int vertex = int(pick(hull->vertices().size()));
        Vector3 outward;
        for (const ConvexHull::Face& face : hull->faces())
            for (int v : face.loop)
                if (v == vertex) outward += face.normal;
        return {hull->vertices()[size_t(vertex)], unit(outward)};
    }
    // Круглые формы: бок — середина образующей; остальное — опорная точка ядра плюс радиус.
    if (element == Side) return {{body.shape->support({1, 0, 0}).x + body.shape->radius, 0, 0}, {1, 0, 0}};
    const Vector3 outward = element == Edge ? unit({1, 1, 0}) : Vector3{0, 1, 0};
    return {body.shape->support(outward) + body.shape->radius * outward, outward};
}

// Кратчайший поворот единичного вектора from в единичный to.
Quaternion turning(Vector3 from, Vector3 to) {
    const Real c = from.dot(to);
    const Vector3 axis = c > Real(-0.999999) ? from.cross(to) : unit(from.cross(std::fabs(from.x) < Real(0.9) ? Vector3{1, 0, 0} : Vector3{0, 1, 0}));
    return c > Real(-0.999999) ? Quaternion{1 + c, axis.x, axis.y, axis.z}.unit() : Quaternion{0, axis.x, axis.y, axis.z};
}

// Ставит тело так, чтобы его элемент смотрел в сторону facing, а точка элемента была в точке at.
void place(Body& body, Spot spot, Vector3 facing, Vector3 at, Quaternion extraTurn) {
    body.orientation = (extraTurn * turning(body.orientation.rotate(spot.outward), facing) * body.orientation).unit();
    body.position = body.position + at - body.pose().point(spot.point);
}

// --- Одна проба ----------------------------------------------------------------------------------

struct Cell {
    int a, b, size, regime;
};

double energy(const World& world) {
    double sum = 0;
    for (const Body& body : world.bodies())
        if (!body.isStatic()) sum += body.kineticEnergy() - world.gravity.dot(body.position) / body.mass.inverseMass;
    return sum;
}

// У шага нет предела числа отрезков, поэтому он может не кончиться. Прибор обрывает шаг после
// kSegmentBudget отрезков и записывает нарушение «шаг без конца»: такой шаг либо не кончается
// вовсе, либо считается секундами. Пробы, оборванные на 200 000 отрезков, не кончались и там.
constexpr int kSegmentBudget = 10000;
struct Endless {};

struct DepthWatch final : StepObserver {
    double deepest = 0;
    int segments = 0; // в текущем шаге
    void after(StepStage stage, const World& world) override {
        if (stage != StepStage::Motion) return;
        deepest = std::fmax(deepest, overlapDepth(world.bodies()[0], world.bodies()[1]));
        if (++segments > kSegmentBudget) throw Endless{};
    }
};

struct Outcome {
    double deepest = 0;   // наибольшее перекрытие, м
    double gain = 0;      // наибольший подъём энергии над уже достигнутым низшим уровнем, доля от её масштаба
    double linear = 0;    // уход импульса пары, доля
    double angular = 0;   // уход момента импульса пары относительно начала координат, доля
    double drift = 0;     // скорость точек тела в конце, м/с
    double slowest = 0;   // самый долгий шаг, с
    int segments = 0;     // наибольшее число отрезков в шаге
    bool hit = false, finite = true, endless = false;
};

Outcome run(World world, Real dt, int steps) {
    DepthWatch watch;
    world.observer = &watch;
    const Body &target = world.bodies()[0], &body = world.bodies()[1];
    const Real reach = body.shape->boundingRadius() + target.shape->boundingRadius();
    const auto momenta = [&](Vector3& linear, Vector3& angular) {
        linear = angular = {};
        for (const Body& b : world.bodies()) linear += b.linearMomentum, angular += b.angularMomentum + b.position.cross(b.linearMomentum);
    };
    const Vector3 before = body.linearMomentum;
    Vector3 linear0, angular0, linear, angular;
    momenta(linear0, angular0);
    const bool conserving = !target.isStatic() && world.gravity.length() == 0;
    const auto finish = [&](Outcome& out) {
        momenta(linear, angular);
        if (conserving) out.linear = (linear - linear0).length() / linear0.length(), out.angular = (angular - angular0).length() / (linear0.length() * reach + angular0.length());
        out.deepest = watch.deepest;
        out.hit = (body.linearMomentum - before).length() > 1e-9 * (before.length() + 1e-300);
    };
    const double scale = body.kineticEnergy() + world.gravity.length() * body.shape->boundingRadius() / body.mass.inverseMass;
    Outcome out;
    double lowest = energy(world); // самый низкий уровень энергии за пробу
    double weight = 0;             // вес подвижных тел, Н
    for (const Body& b : world.bodies())
        if (!b.isStatic()) weight += double(world.gravity.length() / b.mass.inverseMass);
    const double pushOut = weight * 2 * double(kOverlap);
    for (int step = 0; step < steps; ++step) {
        const auto start = std::chrono::steady_clock::now();
        watch.segments = 0;
        try {
            world.step(dt);
        } catch (Endless) {
            out.endless = true, out.segments = kSegmentBudget;
            finish(out);
            return out;
        }
        out.slowest = std::fmax(out.slowest, std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
        if (world.report().stalled) { // движок сам остановил шаг: тела заклинило
            out.endless = true, out.segments = std::max(out.segments, world.report().segments);
            finish(out);
            return out;
        }
        const double now = energy(world);
        out.gain = std::fmax(out.gain, (now - lowest - pushOut) / scale);
        lowest = std::fmin(lowest, now);
        out.segments = std::max(out.segments, world.report().segments);
    }
    finish(out);
    out.drift = body.linearVelocity().length() + body.angularVelocity().length() * body.shape->boundingRadius();
    out.finite = std::isfinite(double(energy(world) + body.position.length() + target.position.length()));
    return out;
}

// Устойчивый покой: плоская грань неподвижного тела, а на ней — грань, бок или шар. Оболочки
// случайны, центр масс у них может оказаться не над гранью, поэтому их покой не судится.
bool mustRest(Cell cell) {
    const Variant a = kVariants[cell.a], b = kVariants[cell.b];
    return cell.regime == 0 && b.element == Face && b.shape != Hull && a.shape != Hull &&
           (a.element == Face || a.element == Side || a.shape == Ball);
}

World scene(Cell cell, int trial, std::uint64_t& random) {
    const Regime& regime = kRegimes[cell.regime];
    const Variant va = kVariants[cell.a], vb = kVariants[cell.b];
    const Real size = kSizes[cell.size];
    World world;
    world.gravity = regime.gravity;
    Body target = make(vb.shape, 1, regime.freeTarget ? 1000 : 0, random);
    Body body = make(va.shape, size, 1000, random);
    const Spot onTarget = spotOf(target, vb.element, random), onBody = spotOf(body, va.element, random);

    // Первая проба — точное совмещение. В остальных — поворот вокруг нормали, смещение в двадцатую
    // долю меньшего тела и наклон: исчезающе малый у лежащих тел и в половине ударов, заметный —
    // в другой половине.
    const Real small = Real(0.05) * std::min(target.shape->boundingRadius(), body.shape->boundingRadius());
    const double twist = trial ? 6.283185307179586 * uniform(random) : 0, around = 6.283185307179586 * uniform(random);
    const double tilt = !trial ? 0 : std::pow(10.0, regime.speed > 0 && trial % 2 == 0 ? -6 + 5.5 * uniform(random) : -16 + 7 * uniform(random));
    const Vector3 offset = trial ? small * Vector3{Real(2 * uniform(random) - 1), 0, Real(2 * uniform(random) - 1)} : Vector3{};
    place(target, onTarget, {0, 1, 0}, {}, {});
    place(body, onBody, {0, -1, 0}, offset + Vector3{0, 10 * (1 + size), 0},
          Quaternion::fromRotationVector(Real(tilt) * Vector3{Real(std::cos(around)), 0, Real(std::sin(around))}) * Quaternion::fromRotationVector({0, Real(twist), 0}));
    // Сдвиг вниз на (расстояние − зазор) оставляет между телами не меньше зазора.
    body.position.y -= closestPoints(*target.shape, target.pose(), *body.shape, body.pose()).distance - regime.clearance;

    body.linearMomentum = {0, -regime.speed / body.mass.inverseMass, 0};
    const Vector3 spin = regime.spinAboutNormal ? Vector3{0, regime.spin, 0} : regime.spin * unit({Real(2 * uniform(random) - 1), Real(2 * uniform(random) - 1), Real(2 * uniform(random) - 1)});
    const Vector3 local = body.orientation.rotateInverse(spin), inverse = body.mass.inverseInertia;
    body.angularMomentum = body.orientation.rotate({local.x / inverse.x, local.y / inverse.y, local.z / inverse.z});
    world.addBody(target);
    world.addBody(body);
    return world;
}

// --- Итог клетки и сводки ------------------------------------------------------------------------

struct Result {
    Outcome worst;
    int trials = 0, passed = 0, missed = 0; // пролёты сквозь тело и честные промахи
    bool judgedRest = false;

    // Пороги. Перекрытие — два допуска движка (kOverlap): внутри отрезка шага глубина вправе
    // превысить допуск ещё на один (EventSearch.cpp, crossingStart). Энергия — 10⁻⁴ её масштаба: для
    // лежащего тела это подъём на 10⁻⁴ его радиуса, 5–125 мкм, того же порядка, что порог
    // перекрытия; порог 10⁻⁶ был бы долями микрона — ниже допуска контакта. Импульсы — 10⁻⁹:
    // округление даёт 10⁻¹⁵, между ними пусто.
    bool overlapped() const { return worst.deepest > 2 * double(kOverlap); }
    bool gained() const { return worst.gain > 1e-4; }
    bool lostMomentum() const { return worst.linear > 1e-9; }
    bool lostSpin() const { return worst.angular > 1e-9; }
    bool drifted() const { return judgedRest && worst.drift > 1e-3; }
    bool bad() const { return passed || !worst.finite || worst.endless || overlapped() || gained() || lostMomentum() || lostSpin() || drifted(); }
};

Result measure(Cell cell, int trials) {
    const Regime& regime = kRegimes[cell.regime];
    std::uint64_t random = 2026 + 7919ull * std::uint64_t(((cell.a * kVariantCount + cell.b) * 3 + cell.size) * kRegimeCount + cell.regime);
    Result result;
    result.judgedRest = mustRest(cell);
    result.trials = trials;
    for (int trial = 0; trial < trials; ++trial) {
        const World world = scene(cell, trial, random);
        const Outcome out = run(world, Real(1) / 60, regime.steps);
        Outcome& worst = result.worst;
        worst.deepest = std::fmax(worst.deepest, out.deepest), worst.gain = std::fmax(worst.gain, out.gain);
        worst.linear = std::fmax(worst.linear, out.linear), worst.angular = std::fmax(worst.angular, out.angular), worst.drift = std::fmax(worst.drift, out.drift);
        worst.slowest = std::fmax(worst.slowest, out.slowest), worst.segments = std::max(worst.segments, out.segments);
        worst.finite = worst.finite && out.finite, worst.endless = worst.endless || out.endless;
        if (regime.speed == 0 || out.hit || out.endless) continue;
        // Удара нет. Эталон идёт шагом, за который самая быстрая точка тела проходит 2 мм.
        const double fastest = regime.speed + regime.spin * world.bodies()[1].shape->boundingRadius();
        const int fine = int(std::ceil(regime.steps / 60.0 * fastest / 0.002));
        (run(world, Real(regime.steps / 60.0 / fine), fine).hit ? result.passed : result.missed) += 1;
    }
    return result;
}

struct Tally {
    std::string name;
    int cells = 0, bad = 0, passed = 0, notFinite = 0, endless = 0, overlapped = 0, gained = 0, lostMomentum = 0, lostSpin = 0, drifted = 0, segments = 0;
    double deepest = 0, gain = 0, slowest = 0;
    void add(const Result& r) {
        cells += 1, bad += r.bad(), passed += r.passed > 0, notFinite += !r.worst.finite, endless += r.worst.endless, overlapped += r.overlapped(), gained += r.gained();
        lostMomentum += r.lostMomentum(), lostSpin += r.lostSpin(), drifted += r.drifted(), segments = std::max(segments, r.worst.segments);
        deepest = std::fmax(deepest, r.worst.deepest), gain = std::fmax(gain, r.worst.gain), slowest = std::fmax(slowest, r.worst.slowest);
    }
};

void print(const char* title, const std::vector<Tally>& rows) {
    std::printf("\n%-30s %6s %7s | %7s %10s %8s %8s %8s %6s %6s %12s | %14s %10s %9s %8s\n", title, "клеток", "плохих", "пролёт", "перекрытие", "энергия", "импульс", "момент", "покой", "числа", "без конца",
                "перекрытие, мм", "энергия", "отрезков", "шаг, мс");
    for (const Tally& t : rows)
        std::printf("%-30s %6d %7d | %7d %10d %8d %8d %8d %6d %6d %12d | %14.4f %10.1e %9d %8.1f\n", t.name.c_str(), t.cells, t.bad, t.passed, t.overlapped, t.gained, t.lostMomentum,
                    t.lostSpin, t.drifted, t.notFinite, t.endless, 1e3 * t.deepest, t.gain, t.segments, 1e3 * t.slowest);
}

std::string variantName(int v) { return std::string(kShapes[kVariants[v].shape]) + ": " + kElements[kVariants[v].element]; }

void printCell(Cell cell, const Result& r) {
    std::printf("%-26s | %-26s | %-17s | %-29s | пролётов %d промахов %d перекрытие %.4f мм энергия %.1e импульс %.1e момент %.1e покой %.1e м/с отрезков %d шаг %.1f мс%s%s\n",
                variantName(cell.a).c_str(), variantName(cell.b).c_str(), kSizeNames[cell.size], kRegimes[cell.regime].name, r.passed, r.missed, 1e3 * r.worst.deepest,
                r.worst.gain, r.worst.linear, r.worst.angular, r.judgedRest ? r.worst.drift : 0.0, r.worst.segments, 1e3 * r.worst.slowest, r.worst.finite ? "" : " НЕ ЧИСЛО", r.worst.endless ? " ШАГ БЕЗ КОНЦА" : "");
}

} // namespace

// Аргументы: проб на клетку (по умолчанию 4); затем либо файл, в конец которого дописываются все
// клетки строками JSON (из них case_table_figures.py строит рисунки отчёта), либо четыре номера:
// вариант A, вариант B, размер, режим — одна клетка по пробам. Печатаются только плохие клетки.
// После файла — необязательный шаг выборки k: считается каждая k-я клетка. Клетки и их случайные
// числа те же, что в полном прогоне, поэтому выборку можно сравнивать с ним клетка в клетку; полный
// прогон идёт полчаса, каждая пятая клетка — шесть минут.
int main(int argc, char** argv) {
    const int trials = argc > 1 ? std::atoi(argv[1]) : 4;
    if (argc > 5) {
        const Cell cell{std::atoi(argv[2]), std::atoi(argv[3]), std::atoi(argv[4]), std::atoi(argv[5])};
        std::uint64_t random = 2026 + 7919ull * std::uint64_t(((cell.a * kVariantCount + cell.b) * 3 + cell.size) * kRegimeCount + cell.regime);
        for (int trial = 0; trial < trials; ++trial) {
            Result one;
            one.worst = run(scene(cell, trial, random), Real(1) / 60, kRegimes[cell.regime].steps);
            one.judgedRest = mustRest(cell);
            std::printf("проба %d, удар %d: ", trial, one.worst.hit);
            printCell(cell, one);
        }
        return 0;
    }
    const int stride = argc > 3 ? std::max(1, std::atoi(argv[3])) : 1;
    std::vector<Cell> cells;
    int number = 0;
    for (int a = 0; a < kVariantCount; ++a)
        for (int b = 0; b < kVariantCount; ++b)
            for (int size = 0; size < 3; ++size)
                for (int regime = 0; regime < kRegimeCount; ++regime)
                    if (number++ % stride == 0) cells.push_back({a, b, size, regime});
    std::vector<Result> results(cells.size());
    // Клетки независимы и считаются параллельно; случайные числа клетки зависят только от её
    // номера, поэтому таблица не зависит от числа потоков. Сторож называет клетки, которые
    // считаются дольше полминуты: шаг без предела числа отрезков может и не кончиться.
    const unsigned threads = std::max(1u, std::thread::hardware_concurrency());
    std::atomic<size_t> next{0}, done{0};
    std::vector<std::atomic<long>> current(threads); // клетка потока или −1
    std::vector<std::thread> workers;
    for (unsigned k = 0; k < threads; ++k)
        workers.emplace_back([&, k] {
            for (size_t i; (i = next++) < cells.size(); ++done) current[k] = long(i), results[i] = measure(cells[i], trials);
            current[k] = -1;
        });
    std::vector<long> watched(threads, -1);
    for (int tick = 0; done < cells.size(); ++tick) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (tick % 30 != 29) continue;
        for (unsigned k = 0; k < threads; ++k) {
            const long now = current[k];
            if (now >= 0 && now == watched[k]) {
                const Cell c = cells[size_t(now)];
                std::fprintf(stderr, "дольше 30 с: клетка %d %d %d %d — %s | %s | %s | %s\n", c.a, c.b, c.size, c.regime, variantName(c.a).c_str(), variantName(c.b).c_str(),
                             kSizeNames[c.size], kRegimes[c.regime].name);
            }
            watched[k] = now;
        }
        std::fprintf(stderr, "готово клеток %zu из %zu\n", size_t(done), cells.size());
    }
    for (std::thread& worker : workers) worker.join();

    std::FILE* rows = argc > 2 ? std::fopen(argv[2], "a") : nullptr;
    Tally total{"вся таблица"};
    std::vector<Tally> byRegime, bySize, byA, byB, byShapes, byElements;
    for (int k = 0; k < kRegimeCount; ++k) byRegime.push_back({kRegimes[k].name});
    for (int k = 0; k < 3; ++k) bySize.push_back({kSizeNames[k]});
    for (int k = 0; k < kVariantCount; ++k) byA.push_back({variantName(k)}), byB.push_back({variantName(k)});
    for (int a = 0; a < 6; ++a)
        for (int b = 0; b < 6; ++b) byShapes.push_back({std::string(kShapes[a]) + " на " + kShapes[b]});
    for (int a = 0; a < 5; ++a)
        for (int b = 0; b < 5; ++b) byElements.push_back({std::string(kElements[a]) + " на " + kElements[b]});
    for (size_t i = 0; i < cells.size(); ++i) {
        const Cell cell = cells[i];
        const Variant a = kVariants[cell.a], b = kVariants[cell.b];
        const Result& r = results[i];
        if (r.bad()) printCell(cell, r);
        if (rows)
            std::fprintf(rows,
                         "{\"a_shape\":\"%s\",\"a_element\":\"%s\",\"b_shape\":\"%s\",\"b_element\":\"%s\",\"size\":\"%s\",\"regime\":\"%s\",\"trials\":%d,"
                         "\"passed\":%d,\"missed\":%d,\"overlap_m\":%.3e,\"energy_gain\":%.3e,\"momentum_loss\":%.3e,\"angular_loss\":%.3e,\"rest_judged\":%d,\"rest_speed\":%.3e,"
                         "\"segments\":%d,\"slowest_step_s\":%.3e,\"finite\":%d,\"endless\":%d,\"bad\":%d}\n",
                         kShapes[a.shape], kElements[a.element], kShapes[b.shape], kElements[b.element], kSizeNames[cell.size], kRegimes[cell.regime].name, r.trials, r.passed,
                         r.missed, r.worst.deepest, r.worst.gain, r.worst.linear, r.worst.angular, int(r.judgedRest), r.worst.drift, r.worst.segments, r.worst.slowest, int(r.worst.finite),
                         int(r.worst.endless), int(r.bad()));
        for (Tally* tally : {&total, &byRegime[size_t(cell.regime)], &bySize[size_t(cell.size)], &byA[size_t(cell.a)], &byB[size_t(cell.b)],
                             &byShapes[size_t(a.shape * 6 + b.shape)], &byElements[size_t(a.element * 5 + b.element)]})
            tally->add(r);
    }
    if (rows) std::fclose(rows);
    print("итог", {total});
    print("по режиму", byRegime);
    print("по отношению размеров", bySize);
    print("чем касается A", byA);
    print("чего касается у B", byB);
    print("фигура A на фигуре B", byShapes);
    print("элемент A на элементе B", byElements);
    std::printf("\nклеток %zu, проб на клетку %d; плохих клеток %d\n", cells.size(), trials, total.bad);
    return total.bad != 0;
}
