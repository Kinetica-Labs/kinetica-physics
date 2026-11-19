// Поиск ошибок алгоритмов статистикой: алгоритм сравнивается с независимым эталоном на тысячах
// поз, ошибка записывается вместе с признаками сцены, и по ковариации видно, от какого признака
// ошибка зависит. Там, где ошибка растёт вместе с признаком, и сидит слабое место.
//
// Эталоны намеренно тупые — полный перебор без тех приёмов, которые проверяются:
//   расстояние между брусками — минимум по всем вершинам и парам рёбер (всегда не меньше истины);
//   зазор проекций тел вдоль найденной нормали (всегда не больше истины);
//   глубина перекрытия — минимум перекрытия проекций всех вершин по всем 15 осям.
//
// Три популяции поз: случайные, почти параллельные грани, далеко от начала координат.

#include <collision/narrowphase/Gjk.h>
#include <collision/narrowphase/Sat.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace kp;

namespace {

struct Sequence {
    unsigned long long state = 88172645463325252ull;
    double next() {
        state = state * 6364136223846793005ull + 1442695040888963407ull;
        return double(state >> 11) / 9007199254740992.0;
    }
    double between(double lo, double hi) { return lo + (hi - lo) * next(); }
};

struct Box {
    Vector3 half;
    Transform pose;
    Vector3 vertex(int i) const { return pose.point({i & 1 ? half.x : -half.x, i & 2 ? half.y : -half.y, i & 4 ? half.z : -half.z}); }
    Vector3 axis(int i) const { return pose.direction({i == 0 ? 1.0 : 0.0, i == 1 ? 1.0 : 0.0, i == 2 ? 1.0 : 0.0}); }
};

// --- Эталон расстояния -----------------------------------------------------------------------
// Точка против сплошного бруска: в осях бруска координаты зажимаются в его размеры.
double pointToBox(Vector3 p, const Box& box) {
    const Vector3 local = box.pose.localDirection(p - box.pose.position);
    const Vector3 clamped{std::clamp(local.x, -box.half.x, box.half.x), std::clamp(local.y, -box.half.y, box.half.y),
                       std::clamp(local.z, -box.half.z, box.half.z)};
    return (local - clamped).length();
}

// Расстояние между отрезками p1q1 и p2q2 (Ericson, «Real-Time Collision Detection», раздел 5.1.9).
double segmentToSegment(Vector3 p1, Vector3 q1, Vector3 p2, Vector3 q2) {
    const Vector3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    const double a = d1.dot(d1), e = d2.dot(d2), f = d2.dot(r), c = d1.dot(r), b = d1.dot(d2);
    const double denominator = a * e - b * b;
    double s = denominator > 0 ? std::clamp((b * f - c * e) / denominator, 0.0, 1.0) : 0;
    double t = (b * s + f) / e;
    if (t < 0) t = 0, s = std::clamp(-c / a, 0.0, 1.0);
    else if (t > 1) t = 1, s = std::clamp((b - c) / a, 0.0, 1.0);
    return (p1 + s * d1 - (p2 + t * d2)).length();
}

double bruteDistance(const Box& a, const Box& b) {
    double best = 1e300;
    for (int i = 0; i < 8; ++i) best = std::min({best, pointToBox(a.vertex(i), b), pointToBox(b.vertex(i), a)});
    // Рёбра бруска: пары вершин, отличающихся одним битом.
    for (int i = 0; i < 8; ++i)
        for (int bitA = 1; bitA < 8; bitA <<= 1)
            if (!(i & bitA))
                for (int j = 0; j < 8; ++j)
                    for (int bitB = 1; bitB < 8; bitB <<= 1)
                        if (!(j & bitB))
                            best = std::min(best, segmentToSegment(a.vertex(i), a.vertex(i | bitA), b.vertex(j), b.vertex(j | bitB)));
    return best;
}

// --- Эталон глубины --------------------------------------------------------------------------
// Перекрытие проекций на ось: min(maxA − minB, maxB − minA). Глубина — наименьшее по 15 осям.
double bruteDepth(const Box& a, const Box& b) {
    double best = 1e300;
    const auto test = [&](Vector3 axis) {
        const double n = axis.length();
        if (n < 1e-9) return;
        double minA = 1e300, maxA = -1e300, minB = 1e300, maxB = -1e300;
        for (int i = 0; i < 8; ++i) {
            const double pa = a.vertex(i).dot(axis) / n, pb = b.vertex(i).dot(axis) / n;
            minA = std::min(minA, pa), maxA = std::max(maxA, pa), minB = std::min(minB, pb), maxB = std::max(maxB, pb);
        }
        best = std::min(best, std::min(maxA - minB, maxB - minA));
    };
    for (int i = 0; i < 3; ++i) test(a.axis(i)), test(b.axis(i));
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) test(a.axis(i).cross(b.axis(j)));
    return best;
}

// --- Выборка ---------------------------------------------------------------------------------
struct Sample {
    double error = 0;                // |алгоритм − эталон|, м
    std::vector<double> features;    // признаки сцены
    std::string description;
};

struct Population {
    std::string name;
    std::vector<std::string> featureNames;
    std::vector<Sample> samples;
};

// Ковариация и корреляция ошибки с каждым признаком. Ошибка берётся в логарифме: она меняется
// на порядки, и интересует, от чего зависит её порядок.
void report(const Population& p, double tolerance) {
    const size_t n = p.samples.size();
    if (n == 0) return;
    std::vector<double> logError(n);
    double meanError = 0, worst = 0;
    size_t worstIndex = 0, above = 0;
    for (size_t i = 0; i < n; ++i) {
        logError[i] = std::log10(p.samples[i].error + 1e-18);
        meanError += logError[i] / double(n);
        if (p.samples[i].error > worst) worst = p.samples[i].error, worstIndex = i;
        above += p.samples[i].error > tolerance;
    }
    double varianceError = 0;
    for (double e : logError) varianceError += (e - meanError) * (e - meanError) / double(n);
    std::printf("\n%s: проб %zu; ошибка: медианный порядок 1e%.1f, наибольшая %.3g м; выше допуска %.0e м: %zu\n", p.name.c_str(), n,
                meanError, worst, tolerance, above);
    std::printf("  %-34s %12s %12s\n", "признак", "ковариация", "корреляция");
    for (size_t f = 0; f < p.featureNames.size(); ++f) {
        double mean = 0, variance = 0, covariance = 0;
        for (const Sample& s : p.samples) mean += s.features[f] / double(n);
        for (size_t i = 0; i < n; ++i) {
            const double d = p.samples[i].features[f] - mean;
            variance += d * d / double(n);
            covariance += d * (logError[i] - meanError) / double(n);
        }
        const double correlation = variance > 0 && varianceError > 0 ? covariance / std::sqrt(variance * varianceError) : 0;
        std::printf("  %-34s %12.4g %12.3f\n", p.featureNames[f].c_str(), covariance, correlation);
    }
    std::printf("  худшая проба: %s\n", p.samples[worstIndex].description.c_str());
}

Quaternion randomRotation(Sequence& random) {
    return Quaternion{random.between(-1, 1), random.between(-1, 1), random.between(-1, 1), random.between(-1, 1)}.unit();
}
// Поворот на малый угол около случайной оси: грани двух брусков почти параллельны.
Quaternion smallRotation(Sequence& random, double angle) {
    const Vector3 axis{random.between(-1, 1), random.between(-1, 1), random.between(-1, 1)};
    return Quaternion::cayley((angle / axis.length()) * axis);
}

double aspect(const Box& box) {
    return std::max({box.half.x, box.half.y, box.half.z}) / std::min({box.half.x, box.half.y, box.half.z});
}
// Насколько параллельны грани двух брусков: наибольший |косинус| между их осями.
double parallelism(const Box& a, const Box& b) {
    double best = 0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) best = std::max(best, std::fabs(a.axis(i).dot(b.axis(j))));
    return best;
}

std::string describe(const Box& a, const Box& b, double algorithm, double reference) {
    char text[400];
    std::snprintf(text, sizeof text,
                  "алгоритм %.12g, эталон %.12g; A half(%.3g %.3g %.3g) pos(%.6g %.6g %.6g); B half(%.3g %.3g %.3g) pos(%.6g %.6g %.6g)",
                  algorithm, reference, a.half.x, a.half.y, a.half.z, a.pose.position.x, a.pose.position.y, a.pose.position.z, b.half.x,
                  b.half.y, b.half.z, b.pose.position.x, b.pose.position.y, b.pose.position.z);
    return text;
}

// Нижняя граница расстояния, которую нельзя оспорить: зазор между проекциями тел на направление n.
// Истинное расстояние не меньше зазора вдоль любого направления.
double gapAlong(Vector3 n, const Box& a, const Box& b) {
    double maxA = -1e300, minB = 1e300;
    for (int i = 0; i < 8; ++i) maxA = std::max(maxA, a.vertex(i).dot(n)), minB = std::min(minB, b.vertex(i).dot(n));
    return minB - maxA;
}

struct Tally {
    int referenceLoose = 0;      // перебор дал больше, чем GJK: эталон неточен (он всегда не меньше истины)
    int gjkAbove = 0;            // GJK дал больше перебора: ошибка GJK
    double falseOverlap = 0;     // наибольший зазор, при котором GJK объявил перекрытие
};

// Одна пара брусков попадает в выборку расстояний (ядра разделены) или глубин (перекрылись).
// Эталоны считаются в осях тела A — иначе вдали от начала координат неточен сам эталон.
//
// Ошибка расстояния — не разность с перебором, а разрыв между ответом GJK и зазором вдоль его же
// нормали. Зазор — нижняя граница истины, а ответ GJK — длина отрезка между точками тел, то есть
// верхняя; если они сошлись, ответ доказан без всякого эталона.
void measure(const Box& worldA, const Box& worldB, double extraFeature, Population& distances, Population& depths, Tally& tally) {
    const ConvexHull pa = ConvexHull::box(worldA.half), pb = ConvexHull::box(worldB.half);
    const ClosestPoints gjk = closestPoints(pa, worldA.pose, pb, worldB.pose);
    const double scale = worldA.pose.position.length() + worldB.pose.position.length() + 1;
    const Box a{worldA.half, {}}, b{worldB.half, worldA.pose.relative(worldB.pose)};
    if (!gjk.coresOverlap) {
        const double brute = bruteDistance(a, b), lower = gapAlong(worldA.pose.localDirection(gjk.normal), a, b);
        tally.referenceLoose += brute > gjk.distance + 1e-9;
        tally.gjkAbove += gjk.distance > brute + 1e-9;
        distances.samples.push_back({std::fabs(gjk.distance - lower),
                                     {std::log10(gjk.distance + 1e-18), parallelism(a, b), std::max(aspect(a), aspect(b)),
                                      double(gjk.iterations), std::log10(scale), extraFeature},
                                     describe(a, b, gjk.distance, lower)});
    } else {
        const double reference = bruteDepth(a, b);
        const SeparatingAxis sat = findSeparatingAxis(pa, worldA.pose, pb, worldB.pose);
        tally.falseOverlap = std::max(tally.falseOverlap, sat.separation);
        depths.samples.push_back({std::fabs(-sat.separation - reference),
                                  {std::log10(std::fabs(reference) + 1e-18), parallelism(a, b), std::max(aspect(a), aspect(b)),
                                   double(sat.kind == SeparatingAxis::Kind::EdgePair), std::log10(scale), extraFeature},
                                  describe(a, b, -sat.separation, reference)});
    }
}

Vector3 randomHalf(Sequence& random) { return {random.between(0.1, 2), random.between(0.1, 2), random.between(0.1, 2)}; }

} // namespace

int main() {
    Sequence random;
    Tally tally[3];
    const std::vector<std::string> distanceFeatures = {"lg расстояния", "параллельность граней", "вытянутость бруска",
                                                       "итераций GJK", "lg удалённости от начала", "признак популяции"};
    const std::vector<std::string> depthFeatures = {"lg глубины", "параллельность граней", "вытянутость бруска",
                                                    "ось из пары рёбер (0/1)", "lg удалённости от начала", "признак популяции"};

    // 1. Случайные позы.
    Population d1{"Расстояние (GJK), случайные позы", distanceFeatures, {}}, p1{"Глубина (SAT), случайные позы", depthFeatures, {}};
    for (int i = 0; i < 20000; ++i) {
        const Box a{randomHalf(random), {{}, randomRotation(random)}};
        const Box b{randomHalf(random), {{random.between(-4, 4), random.between(-4, 4), random.between(-4, 4)}, randomRotation(random)}};
        measure(a, b, 0, d1, p1, tally[0]);
    }

    // 2. Почти параллельные грани: второй брусок повёрнут относительно первого на 10^k радиан,
    //    k от −1 до −12, и стоит почти вплотную. Признак популяции — k.
    Population d2{"Расстояние (GJK), почти параллельные грани", distanceFeatures, {}},
        p2{"Глубина (SAT), почти параллельные грани", depthFeatures, {}};
    for (int i = 0; i < 20000; ++i) {
        const double k = random.between(-12, -1);
        const Quaternion base = randomRotation(random);
        const Box a{randomHalf(random), {{}, base}};
        Box b{randomHalf(random), {{}, (smallRotation(random, std::pow(10.0, k)) * base).unit()}};
        // Сдвиг вдоль одной из осей A на сумму полуразмеров плюс малый зазор или малое перекрытие.
        const int axis = int(random.between(0, 3)) % 3;
        const double halfA = axis == 0 ? a.half.x : axis == 1 ? a.half.y : a.half.z;
        const double halfB = axis == 0 ? b.half.x : axis == 1 ? b.half.y : b.half.z;
        const double gap = (random.next() < 0.5 ? 1 : -1) * std::pow(10.0, random.between(-9, -1));
        b.pose.position = (halfA + halfB + gap) * a.axis(axis) + random.between(-0.05, 0.05) * a.axis((axis + 1) % 3);
        measure(a, b, k, d2, p2, tally[1]);
    }

    // 3. Далеко от начала координат: обе позы сдвинуты на 10^k метров, k от 0 до 8. Признак — k.
    Population d3{"Расстояние (GJK), далеко от начала координат", distanceFeatures, {}},
        p3{"Глубина (SAT), далеко от начала координат", depthFeatures, {}};
    for (int i = 0; i < 20000; ++i) {
        const double k = random.between(0, 8);
        const Vector3 origin = std::pow(10.0, k) * Vector3{random.between(-1, 1), random.between(-1, 1), random.between(-1, 1)};
        const Box a{randomHalf(random), {origin, randomRotation(random)}};
        const Box b{randomHalf(random),
                    {origin + Vector3{random.between(-4, 4), random.between(-4, 4), random.between(-4, 4)}, randomRotation(random)}};
        measure(a, b, k, d3, p3, tally[2]);
    }

    const double tolerance = 1e-9;
    for (const Population* p : {&d1, &p1, &d2, &p2, &d3, &p3}) report(*p, tolerance);
    // Где проходит граница применимости GJK: ошибка по декадам расстояния (признак 0 — lg расстояния).
    std::printf("\nGJK на почти параллельных гранях: ошибка по декадам расстояния\n  %-22s %8s %16s %16s\n", "расстояние, м", "проб",
                "медиана ошибки", "наибольшая");
    for (int decade = -9; decade < -1; ++decade) {
        std::vector<double> errors;
        for (const Sample& sample : d2.samples)
            if (sample.features[0] >= decade && sample.features[0] < decade + 1) errors.push_back(sample.error);
        if (errors.empty()) continue;
        std::sort(errors.begin(), errors.end());
        std::printf("  1e%d … 1e%d %14zu %16.2e %16.2e\n", decade, decade + 1, errors.size(), errors[errors.size() / 2], errors.back());
    }

    std::printf("\nСверка GJK с перебором (допуск %.0e м) и ложные перекрытия:\n", tolerance);
    const char* names[3] = {"случайные позы", "почти параллельные грани", "далеко от начала координат"};
    for (int i = 0; i < 3; ++i)
        std::printf("  %-28s перебор выше GJK (неточен эталон): %d; GJK выше перебора (ошибка GJK): %d; "
                    "наибольший зазор, объявленный перекрытием: %.3g м\n",
                    names[i], tally[i].referenceLoose, tally[i].gjkAbove, tally[i].falseOverlap);
    return 0;
}
