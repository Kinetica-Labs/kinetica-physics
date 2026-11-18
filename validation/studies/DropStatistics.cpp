// Прибор: статистика по случайным падениям куба. Один кадр одного падения — одно наблюдение:
// сколько энергии добавила стадия движения (при нулевом коэффициенте восстановления она обязана
// только отнимать) и каким был кадр. Корреляция прироста с признаками кадра показывает, что его
// вызывает; так же искались ошибки геометрии (docs/experiments/20-geometry-error-analysis.md).

#include <kp/collision/PairFinder.h>
#include <kp/collision/ContactPair.h>
#include <kp/world/World.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace kp;

namespace {

Real cubeEnergy(const World& world) {
    const Body& b = world.bodies()[1];
    return b.kineticEnergy() + Real(9.81) * b.position.y / b.mass.inverseMass;
}

// Сколько энергии добавила и отняла стадия движения за кадр.
struct MotionLedger final : StepObserver {
    Real previous = 0, gained = 0;
    void after(StepStage stage, const World& world) override {
        const Real now = cubeEnergy(world);
        if (stage == StepStage::Motion && now > previous) gained += now - previous;
        previous = now;
    }
};

struct Sample {
    double gain;                     // прирост энергии за кадр, Дж
    std::vector<double> features;
};
const char* kFeatures[] = {"точек контакта в начале кадра", "точек контакта стало больше", "точек контакта стало меньше",
                           "вершин ближе 1 мм к полу", "отрезков шага", "проходов решателя", "скорость центра, м/с",
                           "угловая скорость, рад/с", "шаг с удержанием сближения"};

int contactPoints(const World& world) {
    int count = 0;
    for (const ContactPair& pair : PairFinder(world.bodies(), world.gravity).pairs(0))
        if (pair.inContact()) count += pair.manifold.count;
    return count;
}

double uniform(std::uint64_t& state) { // детерминированный генератор, [0, 1)
    state = state * 6364136223846793005ull + 1442695040888963407ull;
    return double(state >> 11) / double(1ull << 53);
}

// Окружность из n точек радиуса r в плоскости y = height.
void ring(std::vector<Vector3>& points, int n, Real r, Real height) {
    // Поворот на 2π / n повторяется n раз; синус и косинус малого угла — из ряда, этого хватает
    // для расстановки вершин.
    const double angle = 6.283185307179586 / n;
    const double c = std::cos(angle), s = std::sin(angle);
    double x = r, z = 0;
    for (int k = 0; k < n; ++k) {
        points.push_back({Real(x), height, Real(z)});
        const double next = c * x - s * z;
        z = s * x + c * z, x = next;
    }
}

// Тела статистики. Простые формы: cube, brick (вытянутый брусок), capsule, cylinder,
// rounded-cylinder. Через выпуклую оболочку: tetra, wedge (клин), plate (тонкая пластина),
// rod (стержень), cone, gem (двойная пирамида), prism (двенадцатигранная призма), ball
// (многогранный шар), cloud:N (оболочка N случайных точек).
Body bodyByName(const std::string& name, std::uint64_t& random) {
    std::vector<Vector3> points;
    if (name == "cube") return Body::box({0.3, 0.3, 0.3}, 1000);
    if (name == "brick") return Body::box({0.3, 0.1, 0.2}, 1000);
    if (name == "capsule") return Body::capsule(0.25, 0.1, 1000);
    if (name == "cylinder") return Body::cylinder(0.2, 0.15, 1000);
    if (name == "rounded-cylinder") return Body::cylinder(0.2, 0.15, 1000, {}, 0.01);
    if (name == "tetra") points = {{0.3, 0, 0}, {-0.2, 0, 0.25}, {-0.2, 0, -0.25}, {0, 0.4, 0}};
    if (name == "wedge") points = {{-0.3, 0, -0.2}, {0.3, 0, -0.2}, {0, 0.3, -0.2}, {-0.3, 0, 0.2}, {0.3, 0, 0.2}, {0, 0.3, 0.2}};
    if (name == "plate" || name == "rod") {
        const Vector3 h = name == "plate" ? Vector3{0.3, 0.01, 0.2} : Vector3{0.02, 0.3, 0.02};
        for (int i = 0; i < 8; ++i) points.push_back({i & 1 ? h.x : -h.x, i & 2 ? h.y : -h.y, i & 4 ? h.z : -h.z});
    }
    if (name == "cone") ring(points, 24, 0.2, 0), points.push_back({0, 0.4, 0});
    if (name == "gem") ring(points, 6, 0.25, 0), points.push_back({0, 0.3, 0}), points.push_back({0, -0.15, 0});
    if (name == "prism") ring(points, 12, 0.15, -0.2), ring(points, 12, 0.15, 0.2);
    if (name == "ball") {
        while (points.size() < 80) {
            const Vector3 d{2 * uniform(random) - 1, 2 * uniform(random) - 1, 2 * uniform(random) - 1};
            if (d.length() > 0.1 && d.length() < 1) points.push_back((Real(0.2) / d.length()) * d);
        }
    }
    if (name.rfind("cloud:", 0) == 0)
        for (int k = 0; k < std::atoi(name.c_str() + 6); ++k) points.push_back({0.6 * uniform(random) - 0.3, 0.4 * uniform(random) - 0.2, 0.5 * uniform(random) - 0.25});
    if (points.empty()) std::exit((std::fprintf(stderr, "неизвестное тело %s\n", name.c_str()), 2));
    return Body::hull(points, 1000);
}

// Опоры: верхняя плоскость везде на высоте ноль.
Body floorByName(const std::string& name) {
    if (name == "disc") return Body::cylinder(0.5, 5, 0, {0, -0.5, 0}, Real(0.01));
    if (name == "prism") {
        std::vector<Vector3> points;
        ring(points, 24, 5, -1), ring(points, 24, 5, 0);
        return Body::hull(points, 0);
    }
    return Body::box({5, 0.5, 5}, 0, {0, -0.5, 0});
}

} // namespace

// Необязательный аргумент — число шагов в секунду (по умолчанию 60): по зависимости прироста от
// шага видно, какого порядка ошибка его вызывает.
int main(int argc, char** argv) {
    const int rate = argc > 1 ? std::atoi(argv[1]) : 60;
    const Real friction = argc > 2 ? Real(std::atof(argv[2])) : Real(0.5); // второй аргумент — коэффициент трения
    const bool shockPropagation = argc > 3 && std::atoi(argv[3]) != 0;       // третий — проход ударной волны
    const std::string shape = argc > 4 ? argv[4] : "cube";  // четвёртый — тело: см. bodyByName
    const std::string floorKind = argc > 5 ? argv[5] : "plate"; // пятый — опора: plate — плита, disc — торец цилиндра, prism — многогранная призма
    std::vector<Sample> samples;
    std::uint64_t random = 2026;
    double totalGain = 0, totalEnergy = 0, wholeStepGain = 0, flightGain = 0, deepest = 0;
    int dropsWithGain = 0, resting = 0, flat = 0, upright = 0;
    size_t vertexCount = 0, faceCount = 0;
    const int drops = 400, frames = 3 * rate;

    for (int drop = 0; drop < drops; ++drop) {
        World world;
        world.shockPropagation = shockPropagation;
        Body floor = floorByName(floorKind);
        floor.friction = friction;
        world.addBody(floor);
        Body cube = bodyByName(shape, random);
        cube.position = {0, 1 + uniform(random), 0};
        cube.orientation = Quaternion{2 * uniform(random) - 1, 2 * uniform(random) - 1, 2 * uniform(random) - 1, 2 * uniform(random) - 1}.unit();
        if (const auto* corners = dynamic_cast<const ConvexHull*>(cube.shape.get())) vertexCount += corners->vertices().size(), faceCount += corners->faces().size();
        cube.friction = friction;
        world.addBody(cube);
        MotionLedger ledger;
        ledger.previous = cubeEnergy(world);
        world.observer = &ledger;
        totalEnergy += ledger.previous;

        double dropGain = 0;
        for (int frame = 0; frame < frames; ++frame) {
            const int before = contactPoints(world);
            ledger.gained = 0;
            world.step(Real(1) / rate);
            const int after = contactPoints(world);
            const Body& b = world.bodies()[1];
            // Нижняя точка тела — опорная точка формы в направлении «вниз» плюс радиус скругления.
            const Vector3 down = b.pose().localDirection({0, -1, 0});
            const double lowest = b.position.y - (down.dot(b.shape->support(down)) + b.shape->radius);
            const int near = lowest < 1e-3;
            deepest = std::fmax(deepest, -lowest);
            if (world.report().segments == 1) wholeStepGain += ledger.gained;
            if (before == 0 && after == 0 && world.report().segments == 1) flightGain += ledger.gained;
            samples.push_back({ledger.gained,
                               {double(before), double(after > before), double(after < before), double(near), double(world.report().segments),
                                double(world.report().solverIterations), b.linearVelocity().length(), b.angularVelocity().length()}});
            dropGain += ledger.gained;
        }
        totalGain += dropGain;
        const Vector3 axis = world.bodies()[1].pose().direction({0, 1, 0});
        flat += std::fabs(axis.y) < 1e-3;
        upright += std::fabs(axis.y) > 1 - 1e-6;
        resting += world.bodies()[1].linearVelocity().length() < 1e-3 && world.bodies()[1].angularVelocity().length() < 1e-3;
        dropsWithGain += dropGain > 1e-6;
    }

    int framesWithGain = 0;
    double meanGain = 0, largest = 0;
    for (const Sample& s : samples) {
        framesWithGain += s.gain > 1e-6;
        meanGain += s.gain / samples.size();
        largest = std::fmax(largest, s.gain);
    }
    std::printf("тело %s, опора %s, шагов в секунду %d, трение %.2f, падений %d, кадров %zu\n", shape.c_str(), floorKind.c_str(), rate, double(friction), drops, samples.size());
    std::printf("падений с приростом энергии: %d (%.0f %%); кадров с приростом: %d (%.2f %%)\n", dropsWithGain, 100.0 * dropsWithGain / drops,
                framesWithGain, 100.0 * framesWithGain / samples.size());
    std::printf("прирост всего %.3f Дж — %.2e от начальной энергии; наибольший за кадр %.3f Дж\n", totalGain, totalGain / totalEnergy, largest);
    std::printf("из него в шагах без единого удара: %.3f Дж\n", wholeStepGain);
    std::printf("из него в свободном полёте, без единого контакта: %.3f Дж\n", flightGain);
    std::printf("наибольшее проникновение вершины в пол: %.4f мм\n", 1e3 * deepest);
    std::printf("тело: в среднем вершин %.1f, граней %.1f; пришло в покой за %d с: %d из %d\n", double(vertexCount) / drops, double(faceCount) / drops, frames / rate, resting, drops);
    std::printf("ось тела в конце: лежит горизонтально у %d, стоит вертикально у %d\n\n", flat, upright);

    std::printf("%-34s %12s %22s %22s\n", "признак кадра", "корреляция", "среднее в кадрах", "среднее в кадрах");
    std::printf("%-34s %12s %22s %22s\n", "", "с приростом", "с приростом", "без прироста");
    for (size_t f = 0; f < sizeof kFeatures / sizeof *kFeatures; ++f) {
        double mean = 0, with = 0, without = 0;
        for (const Sample& s : samples) {
            mean += s.features[f] / samples.size();
            (s.gain > 1e-6 ? with : without) += s.features[f];
        }
        double covariance = 0, varianceGain = 0, varianceFeature = 0;
        for (const Sample& s : samples) {
            covariance += (s.gain - meanGain) * (s.features[f] - mean);
            varianceGain += (s.gain - meanGain) * (s.gain - meanGain);
            varianceFeature += (s.features[f] - mean) * (s.features[f] - mean);
        }
        std::printf("%-34s %+12.3f %22.3f %22.3f\n", kFeatures[f], covariance / std::sqrt(varianceGain * varianceFeature + 1e-300),
                    with / std::fmax(1, framesWithGain), without / std::fmax(1.0, double(samples.size() - framesWithGain)));
    }
    return 0;
}
