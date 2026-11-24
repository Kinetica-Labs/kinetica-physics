#include <dynamics/time_stepping/CoupledFriction.h>

#include <algorithm>

namespace kp {
namespace {
using Four = std::array<Real, 4>;
using Radii = std::array<Real, 2>;

// Для одной факторизации направляющих нужны пять правых частей: их текущие скорости
// и отклик каждой направляющей на четыре единичных касательных импульса.
struct GuideRhs {
    std::array<Real, 5> values;
    GuideRhs(Real value = 0) { values.fill(value); }
    GuideRhs& operator-=(const GuideRhs& other) {
        for (size_t i = 0; i < values.size(); ++i) values[i] -= other.values[i];
        return *this;
    }
    friend GuideRhs operator*(Real scale, GuideRhs rhs) {
        for (Real& v : rhs.values) v *= scale;
        return rhs;
    }
    friend GuideRhs operator/(GuideRhs rhs, Real scale) {
        for (Real& v : rhs.values) v /= scale;
        return rhs;
    }
};

template<class Velocities> Real velocity(const Bond& row, const Velocities& states) {
    const auto &a = states[size_t(row.a)], &b = states[size_t(row.b)];
    return row.linear.dot(b.v - a.v) + row.angularB.dot(b.w) - row.angularA.dot(a.w);
}

// Столбец M⁻¹ Jᵀ: единичный импульс меняет скорости двух тел. У остальных отклик нулевой.
class UnitResponse {
    struct Motion { Vector3 v, w; };
public:
    UnitResponse(const Bond& row, const StepArray<SolverBody>& states) : a_(size_t(row.a)), b_(size_t(row.b)) {
        const SolverBody &a = states[a_], &b = states[b_];
        first_.v -= a.inverseMass * row.linear;
        first_.w -= a.spinFrom(row.angularA);
        Motion& other = a_ == b_ ? first_ : second_;
        other.v += b.inverseMass * row.linear;
        other.w += b.spinFrom(row.angularB);
    }
    const Motion& operator[](size_t id) const {
        if (id == a_) return first_;
        return id == b_ ? second_ : still_;
    }
private:
    size_t a_, b_;
    Motion first_, second_, still_;
};

struct SlipSystem {
    std::array<Four, 4> response{}; // кг⁻¹
    Four slip{};                    // м/с при текущем накопленном импульсе
};

// Исключаем направляющие B из системы контактных касательных T:
// W = A_TT − A_TB A_BB⁺ A_BT, A = J M⁻¹ Jᵀ. Скорость привода входит только в правую часть.
SlipSystem projectedSystem(const StepArray<SolverBody>& states, const DirectJoints& guides,
                           const std::array<Bond, 4>& tangents) {
    SlipSystem result;
    const auto& bonds = guides.rows();
    StepArray<GuideRhs> raw(bonds.size());
    for (size_t j = 0; j < 4; ++j) {
        const Bond& t = tangents[j];
        const UnitResponse probe(t, states);
        for (size_t i = 0; i < 4; ++i) result.response[i][j] = velocity(tangents[i], probe);
        for (size_t k = 0; k < bonds.size(); ++k) raw[k].values[j] = velocity(bonds[k], probe);
        result.slip[j] = velocity(t, states);
    }
    for (size_t k = 0; k < bonds.size(); ++k)
        raw[k].values[4] = velocity(bonds[k], states) - (bonds[k].driven ? bonds[k].speed : Real(0));
    auto solved = raw;
    guides.solveResponse(solved);
    for (size_t k = 0; k < bonds.size(); ++k) for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) result.response[i][j] -= raw[k].values[i] * solved[k].values[j];
        result.slip[i] -= raw[k].values[i] * solved[k].values[4];
    }
    return result;
}

// Трение и направляющие могут снова сблизить точки, уже разделённые решением нормалей.
// Для каждой такой точки сохраняем нижнюю границу предыдущего прохода, м/с.
struct NormalLimit {
    SoftContact* contact;
    size_t point;
    Bond row;
    Real speed;
};

StepArray<NormalLimit> normalLimits(const std::array<SoftContact*, 2>& pair, const StepArray<SolverBody>& states) {
    StepArray<NormalLimit> limits;
    for (SoftContact* c : pair) for (size_t i = 0; i < c->pointCount(); ++i) {
        const auto p = c->normalRow(i);
        if (p.impulse == 0 && p.gap > 0) continue;
        const Bond row{p.a, p.b, p.normal, p.rA.cross(p.normal), p.rB.cross(p.normal), 0, 0};
        limits.push_back({c, i, row, std::min(Real(0), velocity(row, states))});
    }
    return limits;
}

// Матрица нормалей после исключения направляющих: W = A_NN − A_NB A_BB⁺ A_BN.
StepArray<Real> normalResponse(const StepArray<NormalLimit>& limits, const DirectJoints& guides, const StepArray<SolverBody>& states) {
    const size_t n = limits.size(), m = guides.rows().size();
    StepArray<Real> matrix(n*n), cross(n*m), column(m);
    for (size_t j = 0; j < n; ++j) {
        const UnitResponse probe(limits[j].row, states);
        for (size_t i = 0; i < n; ++i) matrix[i*n+j] = velocity(limits[i].row, probe);
        for (size_t k = 0; k < m; ++k) cross[j*m+k] = velocity(guides.rows()[k], probe);
    }
    for (size_t j = 0; m && j < n; ++j) {
        std::copy_n(cross.begin() + j*m, m, column.begin());
        guides.solveResponse(column);
        for (size_t i = 0; i < n; ++i) for (size_t k = 0; k < m; ++k) matrix[i*n+j] -= cross[i*m+k] * column[k];
    }
    return matrix;
}

void restoreNormals(const StepArray<NormalLimit>& limits, const DirectJoints& guides, StepArray<SolverBody>& states, Real h) {
    constexpr Real tolerance = Real(1e-11); // м/с
    const size_t n = limits.size();
    StepArray<Real> residual(n), correction(n, 0);
    bool closing = false;
    for (size_t i = 0; i < n; ++i) {
        residual[i] = velocity(limits[i].row, states) - limits[i].speed;
        closing |= residual[i] < -tolerance;
    }
    if (!closing) return;
    const auto matrix = normalResponse(limits, guides, states);
    // Дополнительные опорные импульсы только отталкивают. Прежние диски Кулона остаются допустимыми.
    for (int pass = 0; pass < 128; ++pass) {
        Real change = 0;
        for (size_t i = 0; i < n; ++i) {
            const Real diagonal = matrix[i*n+i];
            if (diagonal <= 0) continue;
            const Real added = std::max(-correction[i], -residual[i] / diagonal);
            correction[i] += added;
            for (size_t j = 0; j < n; ++j) residual[j] += matrix[j*n+i] * added;
            change = std::max(change, abs(added * diagonal));
        }
        if (change <= tolerance) break;
    }
    for (size_t i = 0; i < n; ++i) if (correction[i] > 0) {
        const auto& p = limits[i];
        p.contact->setNormalImpulse(states, p.point, p.contact->normalRow(p.point).impulse + correction[i]);
    }
    guides.solve(states, 0, h);
}

Real dot(const Four& a, const Four& b) {
    Real sum = 0;
    for (size_t i = 0; i < 4; ++i) sum += a[i] * b[i];
    return sum;
}

Four onDisks(Four impulse, const Radii& radius) {
    for (size_t i = 0; i < 2; ++i) {
        const Real length = Vector2{impulse[2*i], impulse[2*i+1]}.length();
        if (length <= radius[i]) continue;
        const Real factor = radius[i] / length;
        impulse[2*i] *= factor; impulse[2*i+1] *= factor;
    }
    return impulse;
}

Real rowSumBound(const std::array<Four, 4>& response) {
    Real bound = 0;
    for (const auto& row : response) {
        Real sum = 0;
        for (Real entry : row) sum += abs(entry);
        bound = std::max(bound, sum);
    }
    return bound;
}

Four stickingImpulse(const SlipSystem& system, const Four& free) {
    Four stick{};
    ProfileMatrix mass;
    for (int i = 0; i < 4; ++i) {
        const auto k = size_t(i);
        stick[k] = -free[k];
        mass.addRow(0);
        for (int j = 0; j <= i; ++j) mass.at(i, j) = system.response[k][size_t(j)];
    }
    solveCholesky(mass, stick);
    return stick;
}

// Минимизируем кинетическую энергию на двух дисках Кулона. Градиент по импульсу — это
// скорость скольжения. 1/L даёт невозрастающую энергию при L ≥ λmax(W).
Four frictionImpulse(SlipSystem system, const Four& warm, const Radii& radius) {
    Four free{};
    for (size_t i = 0; i < 4; ++i) free[i] = system.slip[i] - dot(system.response[i], warm);
    // При μN = 0 обе неизвестные равны нулю. Прежний импульс уже вычтен из свободной скорости.
    for (size_t i = 0; i < 4; ++i) if (radius[i / 2] == 0) {
        free[i] = 0;
        for (size_t j = 0; j < 4; ++j) system.response[i][j] = system.response[j][i] = 0;
    }
    const Four stick = stickingImpulse(system, free);
    const Real bound = rowSumBound(system.response);
    const Real tolerance = 64 * kEpsilon * (bound * (radius[0] + radius[1]) + sqrt(dot(free, free)));
    bool rest = onDisks(stick, radius) == stick;
    for (size_t i = 0; rest && i < 4; ++i)
        rest = abs(free[i] + dot(system.response[i], stick)) <= tolerance;
    if (rest) return stick;

    Four impulse = onDisks(warm, radius);
    if (bound == 0) {
        for (size_t i = 0; i < 2; ++i) {
            const Vector2 slip{free[2*i], free[2*i+1]};
            if (slip.length() == 0) continue;
            const auto limit = (-radius[i] / slip.length()) * slip;
            impulse[2*i] = limit.x; impulse[2*i+1] = limit.y;
        }
        return impulse;
    }
    for (int iteration = 0; iteration < 128; ++iteration) {
        Four next;
        for (size_t i = 0; i < 4; ++i) next[i] = impulse[i] - (free[i] + dot(system.response[i], impulse)) / bound;
        next = onDisks(next, radius);
        Real change = 0;
        for (size_t i = 0; i < 4; ++i) change = std::max(change, abs(next[i] - impulse[i]));
        impulse = next;
        if (bound * change <= tolerance) break;
    }
    return impulse;
}
}

std::array<Bond, 4> CoupledFriction::tangentRows() const {
    std::array<Bond, 4> rows{};
    for (size_t i = 0; i < 2; ++i) {
        const auto& c = *pair_[i];
        for (size_t k = 0; k < 2; ++k) {
            const Vector3 t = k == 0 ? c.tangent1_ : c.tangent2_;
            rows[2*i+k] = {c.a_, c.b_, t, c.centreA_.cross(t), c.centreB_.cross(t), 0, 0};
        }
    }
    return rows;
}

bool CoupledFriction::participates(const SoftContact& c) {
    for (const auto& point : c.points_) if (point.impulse != 0 || point.gap <= 0) return true;
    return c.friction_.rub_.x != 0 || c.friction_.rub_.y != 0 || c.friction_.twistImpulse_ != 0
           || c.friction_.rolling_.hasImpulse();
}

bool CoupledFriction::solve(StepArray<SolverBody>& states, const DirectJoints& guides, Real h, Real rollingArm) {
    Radii radius{}, load{}, lever{};
    Four warm{};
    for (size_t i = 0; i < 2; ++i) {
        const auto& c = *pair_[i];
        for (const auto& p : c.points_) { load[i] += p.impulse; lever[i] += p.lever * p.impulse; }
        radius[i] = c.friction_.coefficient * load[i];
        warm[2*i] = c.friction_.rub_.x; warm[2*i+1] = c.friction_.rub_.y;
    }
    if (radius == Radii{} && warm == Four{} && rollingArm == 0
        && pair_[0]->friction_.twistImpulse_ == 0 && pair_[1]->friction_.twistImpulse_ == 0) return false;
    const auto limits = normalLimits(pair_, states);
    for (size_t i = 0; i < 2; ++i) {
        auto& c = *pair_[i];
        c.friction_.solveTwist(states[size_t(c.a_)], states[size_t(c.b_)], c.frame(), lever[i]);
    }
    const auto impulse = frictionImpulse(projectedSystem(states, guides, tangentRows()), warm, radius);
    for (size_t i = 0; i < 2; ++i) {
        auto& c = *pair_[i];
        auto &sa = states[size_t(c.a_)], &sb = states[size_t(c.b_)];
        c.friction_.setTangent(sa, sb, c.frame(), {impulse[2*i], impulse[2*i+1]}, load[i]);
        c.friction_.rolling_.solve(sa, sb, c.tangent1_, c.tangent2_, rollingArm, load[i]);
    }
    guides.solve(states, 0, h);
    restoreNormals(limits, guides, states, h);
    return true;
}
}
