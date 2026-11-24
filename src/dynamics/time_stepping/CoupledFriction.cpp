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

// Минимизируем кинетическую энергию на двух дисках Кулона. Градиент по импульсу — это
// скорость скольжения. 1/L даёт невозрастающую энергию при L ≥ λmax(W).
Four frictionImpulse(const SlipSystem& system, const Four& warm, const Radii& radius) {
    Four free{}, stick{};
    ProfileMatrix mass;
    for (int i = 0; i < 4; ++i) {
        const auto k = size_t(i);
        free[k] = system.slip[k] - dot(system.response[k], warm);
        stick[k] = -free[k];
        mass.addRow(0);
        for (int j = 0; j <= i; ++j) mass.at(i, j) = system.response[k][size_t(j)];
    }
    solveCholesky(mass, stick);
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

bool CoupledFriction::solve(StepArray<SolverBody>& states, const DirectJoints& guides, Real h, Real rollingArm) {
    Radii radius{}, load{}, lever{};
    Four warm{};
    for (size_t i = 0; i < 2; ++i) {
        const auto& c = *pair_[i];
        for (const auto& p : c.points_) { load[i] += p.impulse; lever[i] += p.lever * p.impulse; }
        radius[i] = c.friction_.coefficient * load[i];
        warm[2*i] = c.friction_.rub_.x; warm[2*i+1] = c.friction_.rub_.y;
    }
    if (radius[0] == 0 && radius[1] == 0) return false;
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
    return true;
}
}
