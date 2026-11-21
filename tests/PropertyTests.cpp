// Случайные проверки утверждений, намеченных к доказательству (proofs/): прежде чем доказывать
// утверждение, его дёшево пробуют опровергнуть на десятках тысяч случайных входов. Допуски записаны
// до первого прогона (validation/attempts.jsonl, H36). Входы воспроизводимы: генератор свой, с
// постоянным зерном, без std::random — его распределения у разных библиотек разные.

#include "Check.h"

#include <collision/PairFinder.h>
#include <dynamics/event_driven/ContactSolver.h>
#include <joints/Path.h>
#include <math/Complementarity.h>
#include <math/ProfileMatrix.h>
#include <math/UpdatedCholesky.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

using namespace kp;

namespace {

// SplitMix64 (Steele, Lea, Flood, «Fast splittable pseudorandom number generators», OOPSLA 2014).
struct Random {
    std::uint64_t state;
    std::uint64_t next() {
        std::uint64_t z = state += 0x9E3779B97F4A7C15ull;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    double unit() { return double(next() >> 11) * (1.0 / 9007199254740992.0); } // [0, 1)
    double between(double low, double high) { return low + (high - low) * unit(); }
    int below(int n) { return int(next() % std::uint64_t(n)); }
    Vector3 vector(double size) { return {Real(between(-size, size)), Real(between(-size, size)), Real(between(-size, size))}; }
};

// Итог серии входов: сколько из них нарушило утверждение и наибольшее нарушение.
struct Tally {
    long cases = 0, failures = 0;
    double worst = 0;
    void add(double violation, double tolerance) { ++cases, failures += !(violation <= tolerance), worst = std::fmax(worst, violation); }
};

// --- Разложение Холецкого по профилю (ProfileMatrix.h) -------------------------------------------
// Утверждение: для A = L Lᵀ с профилем first множитель, оставленный solveCholesky, воспроизводит A,
// а ответ x решает A x = b. Для вырожденной A (нулевые диагональные элементы L: строка выражается
// через предыдущие) и совместной правой части b = A z — только A x = b.
// Нарушение — невязка строки, отнесённая к Σ|A|(|x| + |z|): правая часть собрана из слагаемых A z
// и несёт их ошибку округления, даже когда сама после вычитания мала. Допуск 1e-12.
Tally choleskyProperty(std::uint64_t seed, int cases, bool singular) {
    Random random{seed};
    Tally tally;
    for (int c = 0; c < cases; ++c) {
        const int n = 1 + random.below(24);
        const auto at = [n](int i, int j) { return size_t(i * n + j); };
        std::vector<int> first(static_cast<size_t>(n));
        std::vector<Real> lower(size_t(n * n), 0), A(size_t(n * n), 0), b(size_t(n), 0), z(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) {
            first[size_t(i)] = random.below(i + 1);
            for (int j = first[size_t(i)]; j < i; ++j) lower[at(i, j)] = Real(random.between(-1, 1));
            lower[at(i, i)] = singular && random.unit() < 0.3 ? Real(0) : Real(random.between(0.5, 2));
        }
        for (int i = 0; i < n; ++i)
            for (int j = 0; j <= i; ++j) {
                for (int k = 0; k <= j; ++k) A[at(i, j)] += lower[at(i, k)] * lower[at(j, k)];
                A[at(j, i)] = A[at(i, j)];
            }
        ProfileMatrix a;
        for (int i = 0; i < n; ++i) {
            a.addRow(first[size_t(i)]);
            for (int j = first[size_t(i)]; j <= i; ++j) a.at(i, j) = A[at(i, j)];
            z[size_t(i)] = Real(random.between(-1, 1));
            for (int k = 0; k < n; ++k) b[size_t(k)] += A[at(k, i)] * z[size_t(i)];
        }
        std::vector<Real> x = b;
        solveCholesky(a, x);
        double violation = 0;
        for (int i = 0; i < n; ++i) {
            double residual = -double(b[size_t(i)]), scale = 0;
            for (int j = 0; j < n; ++j) residual += double(A[at(i, j)] * x[size_t(j)]), scale += std::fabs(double(A[at(i, j)])) * (std::fabs(double(x[size_t(j)])) + std::fabs(double(z[size_t(j)])));
            if (scale > 0) violation = std::fmax(violation, std::fabs(residual) / scale);
            for (int j = first[size_t(i)]; j <= i && !singular; ++j) {
                double product = 0;
                for (int k = std::max(first[size_t(i)], first[size_t(j)]); k <= j; ++k) product += double(a.at(i, k) * a.at(j, k));
                violation = std::fmax(violation, std::fabs(product - double(A[at(i, j)])) / std::sqrt(double(A[at(i, i)] * A[at(j, j)])));
            }
        }
        tally.add(violation, 1e-12);
    }
    return tally;
}

// --- Обновляемое разложение Холецкого (UpdatedCholesky.h) ----------------------------------------
// Утверждение: после любой последовательности добавлений и удалений строк ответ x решает A_S x = b, где
// A_S — подматрица A на нынешнем наборе линий в порядке их входа. A = L Lᵀ + I с профилем: наименьшее
// собственное число не меньше единицы, зависимых строк нет. Профиль новой строки — от самой ранней
// связанной с ней линии набора, как в SetFactor. Нарушение и допуск — как у choleskyProperty: 1e-12.
double updatedResidual(Random& random, const UpdatedCholesky& factor, const std::vector<Real>& A, int n, const std::vector<int>& lines) {
    const size_t m = lines.size();
    const auto a = [&](size_t i, size_t j) { return A[size_t(lines[i] * n + lines[j])]; };
    StepArray<Real> z(m), b(m, 0);
    for (size_t j = 0; j < m; ++j) {
        z[j] = Real(random.between(-1, 1));
        for (size_t i = 0; i < m; ++i) b[i] += a(i, j) * z[j];
    }
    StepArray<Real> x = b;
    factor.solve(x);
    double violation = 0;
    for (size_t i = 0; i < m; ++i) {
        double residual = -double(b[i]), scale = 0;
        for (size_t j = 0; j < m; ++j) residual += double(a(i, j) * x[j]), scale += std::fabs(double(a(i, j))) * (std::fabs(double(x[j])) + std::fabs(double(z[j])));
        if (scale > 0) violation = std::fmax(violation, std::fabs(residual) / scale);
    }
    return violation;
}

Tally updatedCholeskyProperty(std::uint64_t seed, int cases) {
    Random random{seed};
    Tally tally;
    for (int c = 0; c < cases; ++c) {
        const int n = 2 + random.below(23);
        const auto at = [n](int i, int j) { return size_t(i * n + j); };
        std::vector<Real> lower(size_t(n * n), 0), A(size_t(n * n), 0);
        for (int i = 0; i < n; ++i) {
            for (int j = random.below(i + 1); j < i; ++j) lower[at(i, j)] = Real(random.between(-1, 1));
            lower[at(i, i)] = Real(random.between(0.5, 2));
        }
        for (int i = 0; i < n; ++i)
            for (int j = 0; j <= i; ++j) {
                for (int k = 0; k <= j; ++k) A[at(i, j)] += lower[at(i, k)] * lower[at(j, k)];
                A[at(j, i)] = A[at(i, j)] += i == j ? Real(1) : Real(0);
            }
        UpdatedCholesky factor;
        std::vector<int> lines; // линии набора в порядке разложения
        double violation = 0;
        for (int edit = 0; edit < 40; ++edit) {
            if (!lines.empty() && (int(lines.size()) == n || random.unit() < 0.4)) {
                const int place = random.below(int(lines.size()));
                factor.remove(place), lines.erase(lines.begin() + place);
            } else {
                int line = random.below(n), first = int(lines.size());
                while (std::find(lines.begin(), lines.end(), line) != lines.end()) line = (line + 1) % n;
                for (int p = int(lines.size()); p-- > 0;)
                    if (A[at(line, lines[size_t(p)])] != 0) first = p;
                StepArray<Real> row;
                for (size_t p = size_t(first); p < lines.size(); ++p) row.push_back(A[at(line, lines[p])]);
                factor.append(first, row, A[at(line, line)]), lines.push_back(line);
            }
            violation = std::fmax(violation, updatedResidual(random, factor, A, n, lines));
        }
        tally.add(violation, 1e-12);
    }
    return tally;
}

// Строка, повторяющая прежнюю, пропускается: её неизвестная — ноль, остальные решают систему без неё.
void updatedCholeskySkipsDependentRow() {
    UpdatedCholesky factor;
    factor.append(0, {}, 2), factor.append(0, {1}, 3), factor.append(0, {2, 1}, 2); // третья строка равна первой
    StepArray<Real> x = {4, 5, 4};
    factor.solve(x);
    CHECK_NEAR(x[0], Real(1.4), Real(1e-12));
    CHECK_NEAR(x[1], Real(1.2), Real(1e-12));
    CHECK(x[2] == 0, "неизвестная зависимой строки %g, а не ноль", double(x[2]));
}

// --- Задача дополнительности (Complementarity.h) -------------------------------------------------
// Утверждение: ответ solveComplementarity удовлетворяет λ ≥ 0, w = A λ + q ≥ 0, λᵢ wᵢ = 0.
// A = J Jᵀ, как у контакта (J — строки точек, M = 1), диагональ умножена на 1 + ridge:
//  независимые строки, ridge = 0,01 — ни один ведущий элемент не падает ниже kDependent; q любое;
//  зависимые строки (четвёртая точка на грани выражается через три), ridge = 0,3 — так задачу
//    ставит движок: проксимальная поправка SoftContact::solveNormals; q = J v;
//  зависимые строки, ridge = 0 — дыра: утверждение неверно (см. propertyTests).
// Нарушение — наибольшее из −wᵢ и (при λᵢ > 0) |wᵢ|, отнесённое к Σ|A|λ + |q|; допуск 1e-9.
std::vector<Real> contactLikeRows(Random& random, int n, int m, int rank) {
    std::vector<Real> J(size_t(n * m), 0);
    for (int i = 0; i < n; ++i)
        for (int term = 0; term < (i < rank ? 1 : 3); ++term) {
            const int source = random.below(rank);
            const Real weight = Real(random.between(-1, 1));
            for (int k = 0; k < m; ++k) J[size_t(i * m + k)] += i < rank ? Real(random.between(-1, 1)) : weight * J[size_t(source * m + k)];
        }
    return J;
}

Tally complementarityProperty(std::uint64_t seed, int cases, bool dependent, Real ridge) {
    Random random{seed};
    Tally tally;
    for (int c = 0; c < cases; ++c) {
        const int n = 1 + random.below(8);
        const int rank = dependent ? 1 + random.below(n) : n;
        const int m = dependent ? rank + 3 + random.below(4) : n + random.below(4);
        const std::vector<Real> J = contactLikeRows(random, n, m, rank);
        std::vector<Real> A(size_t(n * n), 0), q(size_t(n), 0), lambda(static_cast<size_t>(n)), v(static_cast<size_t>(m));
        for (Real& value : v) value = Real(random.between(-1, 1));
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j)
                for (int k = 0; k < m; ++k) A[size_t(i * n + j)] += J[size_t(i * m + k)] * J[size_t(j * m + k)];
            for (int k = 0; k < m; ++k) q[size_t(i)] += J[size_t(i * m + k)] * v[size_t(k)];
            A[size_t(i * n + i)] *= 1 + ridge;
            if (!dependent) q[size_t(i)] = Real(random.between(-1, 1));
            lambda[size_t(i)] = random.unit() < 0.5 ? Real(0) : Real(random.unit()); // тёплый старт: произвольный
        }
        solveComplementarity(A, q, lambda);
        double violation = 0, scale = 0;
        for (int i = 0; i < n; ++i) {
            double w = double(q[size_t(i)]), size = std::fabs(double(q[size_t(i)]));
            for (int j = 0; j < n; ++j) w += double(A[size_t(i * n + j)] * lambda[size_t(j)]), size += std::fabs(double(A[size_t(i * n + j)] * lambda[size_t(j)]));
            scale = std::fmax(scale, size);
            violation = std::fmax(violation, lambda[size_t(i)] > 0 ? std::fabs(w) : -w);
            if (lambda[size_t(i)] < 0) violation = HUGE_VAL;
        }
        tally.add(scale > 0 ? violation / scale : violation, 1e-9);
    }
    return tally;
}

// --- Удар (ContactSolver::solveImpacts) ----------------------------------------------------------
// Утверждения для свободной группы тел (неподвижных нет):
//  linear, angular — сумма импульсов и сумма моментов импульса относительно начала координат не
//                    меняются: уход, отнесённый к Σ|p| и Σ(|L| + |r||p|), допуск 1e-12;
//  speed           — в каждой касающейся точке скорость сближения после удара не ниже цели
//                    (−e v при сближении, 0 иначе): недобор, м/с, допуск 1e-6;
//  energy          — кинетическая энергия не выросла: прирост, отнесённый к энергии до удара,
//                    допуск 1e-12. Верно только для неупругого удара (см. propertyTests).
struct ImpactTallies {
    Tally linear, angular, speed, energy;
    long struck = 0; // в скольких входах удар был
};

// Половина высоты тела вдоль его оси Y — на ней стоит следующее тело столбика.
struct Solid {
    Body body;
    Real halfHeight = 0;
};

Solid randomSolid(Random& random, bool flat, bool bounce) {
    const Real density = Real(random.between(300, 3000));
    const auto size = [&] { return Real(random.between(0.1, 0.5)); };
    Solid solid;
    const int kind = random.below(flat ? 2 : 4);
    const Vector3 half{size(), size(), size()};
    solid.halfHeight = half.y;
    if (kind == 0) solid.body = Body::box(half, density);
    else if (kind == 1) solid.body = Body::cylinder(half.y, half.x, density);
    else if (kind == 2) solid.body = Body::sphere(half.x, density);
    else solid.body = Body::capsule(half.y, half.x, density);
    solid.body.friction = Real(random.between(0, 1));
    solid.body.restitution = bounce ? Real(random.unit()) : Real(0);
    return solid;
}

void throwBody(Random& random, Body& body) {
    body.linearMomentum = (1 / body.mass.inverseMass) * random.vector(2);
    const Vector3 spin = random.vector(3), inverse = body.mass.inverseInertia;
    body.angularMomentum = body.orientation.rotate({spin.x / inverse.x, spin.y / inverse.y, spin.z / inverse.z});
}

// Два тела в общем положении, сведённые до зазора внутри полосы касания: одна точка контакта.
std::vector<Body> touchingPair(Random& random, bool bounce) {
    std::vector<Body> bodies{randomSolid(random, false, bounce).body, randomSolid(random, false, bounce).body};
    for (Body& body : bodies) body.orientation = Quaternion::fromRotationVector(random.vector(3));
    Vector3 direction = random.vector(1);
    if (direction.length() < Real(0.1)) direction = {0, 1, 0};
    bodies[1].position = ((bodies[0].boundingRadius() + bodies[1].boundingRadius() + 1) / direction.length()) * direction;
    // Сдвиг вдоль линии ближайших точек на t уменьшает расстояние ровно на t.
    const Separation apart = separationOf(bodies[0], bodies[1]);
    bodies[1].position -= (apart.distance - Real(random.between(0, 0.5)) * kContactDistance) * apart.normal;
    return bodies;
}

// Столбик из двух–трёх брусков и цилиндров, стоящих плашмя друг на друге: площадки из многих точек.
std::vector<Body> flushStack(Random& random, bool bounce) {
    std::vector<Body> bodies;
    Real below = 0;
    for (int k = 0, count = 2 + random.below(2); k < count; ++k) {
        Solid solid = randomSolid(random, true, bounce);
        if (k == 0) solid.body.orientation = Quaternion::fromRotationVector(random.vector(3));
        else {
            const Body& base = bodies.back();
            const Vector3 seat{Real(random.between(-0.05, 0.05)), below + solid.halfHeight + Real(random.between(0, 0.5)) * kContactDistance, Real(random.between(-0.05, 0.05))};
            solid.body.orientation = base.orientation * Quaternion::fromRotationVector({0, Real(random.between(-3, 3)), 0});
            solid.body.position = base.position + base.orientation.rotate(seat);
        }
        below = solid.halfHeight;
        bodies.push_back(solid.body);
    }
    return bodies;
}

struct Momenta {
    Vector3 linear, angular;
    double linearScale = 0, angularScale = 0, energy = 0;
};

Momenta momentaOf(const std::vector<Body>& bodies) {
    Momenta total;
    for (const Body& body : bodies) {
        total.linear += body.linearMomentum;
        total.angular += body.angularMomentum + body.position.cross(body.linearMomentum);
        total.linearScale += double(body.linearMomentum.length());
        total.angularScale += double(body.angularMomentum.length() + body.position.length() * body.linearMomentum.length());
        total.energy += double(kineticEnergyOf(body));
    }
    return total;
}

Real closingSpeed(const std::vector<Body>& bodies, const ContactPair& pair, const ContactPoint& point) {
    const Vector3 at = Real(0.5) * (point.onA + point.onB);
    return pair.manifold.normal.dot(pointVelocity(bodies[size_t(pair.b)], at) - pointVelocity(bodies[size_t(pair.a)], at));
}

// Этап удара точного шага (ExactStep::resolveImpacts): два прохода, второй — с трением в центре
// давления точек, на которые пришёлся удар.
bool resolveImpacts(std::vector<Body>& bodies, const std::vector<ContactPair>& pairs) {
    const std::vector<Body> before = bodies;
    const ContactForces none;
    ContactSolver first(bodies, pairs, Vector3{}, 0, &none);
    first.solveImpacts();
    const ContactForces struck = first.pressures();
    bool any = false;
    for (const auto& [id, impulse] : struck) any = any || impulse.normal > 0;
    if (!any) return false;
    bodies = before;
    ContactSolver(bodies, pairs, Vector3{}, 0, &struck).solveImpacts();
    return true;
}

ImpactTallies impactProperty(std::uint64_t seed, int cases, bool stacked, bool bounce) {
    Random random{seed};
    ImpactTallies tally;
    for (int c = 0; c < cases; ++c) {
        std::vector<Body> bodies = stacked ? flushStack(random, bounce) : touchingPair(random, bounce);
        for (Body& body : bodies) throwBody(random, body);
        const std::vector<Body> before = bodies;
        const std::vector<ContactPair> pairs = PairFinder(bodies, Vector3{}).pairs(0);
        tally.struck += resolveImpacts(bodies, pairs);
        const Momenta was = momentaOf(before), now = momentaOf(bodies);
        tally.linear.add(double((now.linear - was.linear).length()) / was.linearScale, 1e-12);
        tally.angular.add(double((now.angular - was.angular).length()) / was.angularScale, 1e-12);
        tally.energy.add((now.energy - was.energy) / was.energy, 1e-12);
        double shortfall = 0;
        for (const ContactPair& pair : pairs)
            for (int k = 0; k < pair.manifold.count && pair.inContact(); ++k) {
                const ContactPoint& point = pair.manifold.points[size_t(k)];
                if (point.separation > kContactDistance) continue;
                const Real approach = closingSpeed(before, pair, point);
                const Real bounce = std::max(before[size_t(pair.a)].restitution, before[size_t(pair.b)].restitution);
                shortfall = std::fmax(shortfall, double((approach < 0 ? -bounce * approach : Real(0)) - closingSpeed(bodies, pair, point)));
            }
        tally.speed.add(shortfall, 1e-6);
    }
    return tally;
}

// --- Кватернион из тройки осей (Quaternion::fromAxes) -----------------------------------------------
// Утверждение: оси, повёрнутые единичным кватернионом q, возвращают q или −q. Допуск 1e-12.
Tally fromAxesProperty(std::uint64_t seed, int cases) {
    Random random{seed};
    Tally tally;
    for (int c = 0; c < cases; ++c) {
        const Quaternion q = Quaternion{Real(random.between(-1, 1)), Real(random.between(-1, 1)), Real(random.between(-1, 1)), Real(random.between(-1, 1))}.unit();
        const Quaternion back = Quaternion::fromAxes(q.rotate({1, 0, 0}), q.rotate({0, 1, 0}), q.rotate({0, 0, 1}));
        const double same = std::fabs(double(back.w - q.w)) + std::fabs(double(back.x - q.x)) + std::fabs(double(back.y - q.y)) + std::fabs(double(back.z - q.z));
        const double opposite = std::fabs(double(back.w + q.w)) + std::fabs(double(back.x + q.x)) + std::fabs(double(back.y + q.y)) + std::fabs(double(back.z + q.z));
        tally.add(std::fmin(same, opposite), 1e-12);
    }
    return tally;
}

// --- Кривая пути (joints/Path.h) -------------------------------------------------------------------
// Случайная пологая кривая: точки по окружности радиуса 1 м через равные углы (замкнутая — весь круг,
// открытая — три четверти), каждая сдвинута не больше чем на 0,3 шага между точками: без петель и возвратов.
struct GentlePath {
    Path path;
    double spacing; // шаг между точками, м
    std::vector<Vector3> through;
};
GentlePath gentlePath(Random& random) {
    const int n = 4 + random.below(37);
    const bool closed = random.below(2) == 1;
    const double step = (closed ? 2.0 : 1.5) * 3.14159265358979323846 / (closed ? n : n - 1);
    std::vector<Vector3> through;
    for (int k = 0; k < n; ++k) through.push_back(Vector3{Real(std::cos(step * k)), 0, Real(std::sin(step * k))} + random.vector(0.3 * step / 1.7320508));
    return {Path(through, closed), step, through};
}

struct PathTallies {
    Tally nearest, foot, length, unit, across, twist, seam, through, bend;
};

// Утверждение: кривая проходит через заданные точки (1e-12 м), а кривизна по обе стороны заданной точки
// одна и та же (1e-6 1/м) — вторая производная сплайна непрерывна.
void checkKnot(Random& random, const GentlePath& curve, const std::vector<Vector3>& through, PathTallies& tallies) {
    const Path& path = curve.path;
    const int k = path.closed() ? random.below(int(through.size())) : 1 + random.below(int(through.size()) - 2);
    tallies.through.add(double((path.point(Real(k)) - through[size_t(k)]).length()), 1e-12);
    const Real before = k == 0 ? path.span() - Real(1e-9) : Real(k) - Real(1e-9);
    tallies.bend.add(double((path.curvature(Real(k) + Real(1e-9)) - path.curvature(before)).length()), 1e-6);
}

// Утверждение: для точки x = P(u₀) + d ближайшая точка кривой не дальше |d| (сама P(u₀) на таком
// расстоянии), и в ней (P − x) · T = 0 — кроме концов открытой кривой, от которых кривая уходит прочь от x.
// |d| — до 0,2 шага точек. Допуски 1e-12 м и 1e-9 м.
void checkClosest(Random& random, const GentlePath& curve, PathTallies& tallies) {
    const Path& path = curve.path;
    const Vector3 shift = random.vector(0.2 * curve.spacing / 1.7320508), x = path.point(Real(random.between(0, double(path.span())))) + shift;
    const Real u = path.closest(x);
    const Vector3 gap = path.point(u) - x;
    const double along = double(gap.dot(path.tangent(u)));
    const bool atEnd = !path.closed() && ((u <= 0 && along >= 0) || (u >= path.span() && along <= 0));
    tallies.nearest.add(double(gap.length() - shift.length()), 1e-12);
    tallies.foot.add(atEnd ? 0 : std::fabs(along), 1e-9);
}

// Утверждение: нормаль единичная, поперёк касательной и идёт без кручения. Кручение между u и u + δ —
// осевая часть поворота, переводящего тройку (T, N, B) в следующую: ½ (N₂ · B₁ − N₁ · B₂), B = T × N.
// Допуск 1e-2 рад на метр; у замкнутой кривой к нему добавлено распределённое по ней (не больше π на её
// длину). Шаг δ = 1/1024 отрезка: у самого переноса без кручения поворот на конечном шаге имеет осевую
// часть, она убывает вместе с шагом (при δ = 1/64 — до 1,1e-2 рад/м, при 1/1024 — 3e-4).
// У замкнутой кривой нормаль смыкается: в конце круга та же, что в начале, до 1e-9.
void checkNormal(Random& random, const Path& path, PathTallies& tallies) {
    const Real u = Real(random.between(0, double(path.span()) - (path.closed() ? 0 : 1.0 / 1024))), next = u + Real(1) / 1024;
    const Vector3 t1 = path.tangent(u), n1 = path.normal(u), t2 = path.tangent(next), n2 = path.normal(next);
    tallies.unit.add(std::fabs(double(n1.length()) - 1), 1e-12);
    tallies.across.add(std::fabs(double(n1.dot(t1))), 1e-12);
    double travelled = double(path.length(next) - path.length(u));
    if (travelled < 0) travelled += double(path.length()); // замкнутая кривая: переход через начало
    const double twist = 0.5 * std::fabs(double(n2.dot(t1.cross(n1)) - n1.dot(t2.cross(n2))));
    tallies.twist.add(twist - ((path.closed() ? 3.14159265358979323846 / double(path.length()) : 0) + 1e-2) * travelled, 0);
    if (path.closed()) tallies.seam.add(double((path.normal(0) - path.normal(path.span() * Real(1 - 1e-12))).length()), 1e-9);
}

// Длина дуги до случайного u против сумм хорд: на каждом отрезке кривой 128 и 256 хорд и уточнение
// Ричардсона (ошибка суммы хорд — порядка квадрата шага). Допуск 1e-6 м.
double lengthError(Random& random, const Path& path) {
    const double u = random.between(0, double(path.span()));
    double reference = 0;
    for (int segment = 0; segment < u; ++segment) {
        const double from = segment, span = std::fmin(segment + 1.0, u) - from;
        double sum[2] = {0, 0};
        for (int fine = 0; fine < 2; ++fine)
            for (int k = 0, pieces = 128 << fine; k < pieces; ++k) sum[fine] += double((path.point(Real(from + span * (k + 1) / pieces)) - path.point(Real(from + span * k / pieces))).length());
        reference += (4 * sum[1] - sum[0]) / 3;
    }
    return std::fabs(double(path.length(Real(u))) - reference);
}

PathTallies pathProperty(std::uint64_t seed, int cases) {
    Random random{seed};
    PathTallies tallies;
    for (int c = 0; c < cases; ++c) {
        const GentlePath curve = gentlePath(random);
        checkClosest(random, curve, tallies);
        checkNormal(random, curve.path, tallies);
        checkKnot(random, curve, curve.through, tallies);
        if (c % 50 == 0) tallies.length.add(lengthError(random, curve.path), 1e-6);
    }
    return tallies;
}

// Плоское кольцо: нормаль, перенесённая без кручения, не поворачивается относительно плоскости — её
// составляющая вдоль нормали плоскости постоянна.
void planarPathKeepsNormal() {
    std::vector<Vector3> ring;
    for (int k = 0; k < 8; ++k) ring.push_back({Real(std::cos(0.785398163397448 * k)), 0, Real(std::sin(0.785398163397448 * k))});
    const Path path(ring, true);
    double out = 0;
    for (int k = 0; k < 800; ++k) out = std::fmax(out, std::fabs(double(path.normal(Real(0.01 * k)).y - path.normal(0).y)));
    CHECK(out < 1e-12, "нормаль плоского кольца повернулась относительно плоскости на %.2e", out);
    // То же кольцо — окружность радиуса 1 м через 8 точек: кривизна и длина близки к её собственным.
    double bend = 0;
    for (int k = 0; k < 800; ++k) bend = std::fmax(bend, std::fabs(double(path.curvature(Real(0.01 * k)).length()) - 1));
    CHECK(bend < 0.06, "кривизна кольца через 8 точек отличается от 1 на %.3f", bend);
    CHECK_NEAR(double(path.length()), 6.283185307179586, 5e-3);
}

void expectNone(const char* statement, const Tally& tally) {
    CHECK(tally.failures == 0, "%s: нарушений %ld из %ld, наибольшее %.3g", statement, tally.failures, tally.cases, tally.worst);
}

// Дыра: утверждение, про которое известно, что оно неверно. Проверка держит это знание: если
// нарушения исчезли, алгоритм дыру закрыл — и утверждение пора переносить в доказуемые.
void expectSome(const char* hole, const Tally& tally) { CHECK(tally.failures > 0, "%s: нарушений нет на %ld входах — дыра закрыта?", hole, tally.cases); }

} // namespace

void propertyTests() {
    expectNone("Холецкий по профилю, положительно определённая матрица", choleskyProperty(1, 100000, false));
    expectNone("Холецкий по профилю, вырожденная матрица с совместной правой частью", choleskyProperty(2, 100000, true));
    expectNone("обновляемый Холецкий: добавления и удаления строк", updatedCholeskyProperty(5, 20000));
    updatedCholeskySkipsDependentRow();
    expectNone("кватернион из тройки осей возвращает поворот", fromAxesProperty(12, 100000));
    const PathTallies path = pathProperty(11, 10000);
    expectNone("путь: ближайшая точка не дальше исходного смещения", path.nearest);
    expectNone("путь: в ближайшей точке (P − x) · T = 0", path.foot);
    expectNone("путь: длина дуги против сумм хорд", path.length);
    expectNone("путь: нормаль единичная", path.unit);
    expectNone("путь: нормаль поперёк касательной", path.across);
    expectNone("путь: нормаль идёт без кручения", path.twist);
    expectNone("путь: нормаль замкнутой кривой смыкается", path.seam);
    expectNone("путь: кривая проходит через заданные точки", path.through);
    expectNone("путь: кривизна в заданной точке не рвётся", path.bend);
    planarPathKeepsNormal();
    expectNone("задача дополнительности, независимые строки", complementarityProperty(3, 100000, false, Real(0.01)));
    expectNone("задача дополнительности, зависимые строки с проксимальной поправкой", complementarityProperty(4, 100000, true, Real(0.3)));
    // Без поправки знак остатка w на зависимой строке — шум округления: строка то входит в набор
    // давящих, то выходит, и перебор исчерпывает свои 2ⁿ кругов (24 % входов).
    expectSome("дыра: задача дополнительности с вырожденной матрицей", complementarityProperty(4, 20000, true, 0));
    for (int series = 0; series < 4; ++series) {
        const bool stacked = series & 1, bounce = series & 2;
        const ImpactTallies impact = impactProperty(std::uint64_t(5 + series), 10000, stacked, bounce);
        CHECK(impact.struck > impact.linear.cases / 4, "ударов %ld из %ld входов", impact.struck, impact.linear.cases);
        expectNone("удар сохраняет импульс", impact.linear);
        expectNone("удар сохраняет момент импульса", impact.angular);
        expectNone("удар останавливает сближение в касающихся точках", impact.speed);
        // Закон Ньютона (v → −e v в каждой точке) энергию не бережёт: у одной точки с трением она
        // растёт до 8 %, у площадки из нескольких точек — до 60 %. Неупругий удар её не добавляет.
        if (bounce) expectSome("дыра: удар с отскоком добавляет энергию", impact.energy);
        else expectNone("неупругий удар не добавляет энергии", impact.energy);
    }
}
