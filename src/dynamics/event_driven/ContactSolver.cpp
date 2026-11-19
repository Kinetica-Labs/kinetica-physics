#include <dynamics/event_driven/ContactSolver.h>

#include <algorithm>
#include <array>
#include <limits>

namespace kp {
namespace {

constexpr int kMaxIterations = 200; // чередований «нормали — трение»
constexpr Real kIdle = ContactRows::kIdle;

} // namespace

ContactSolver::ContactSolver(std::vector<Body>& bodies, const std::vector<ContactPair>& pairs, Vector3 up, Real restBounceSpeed, const ContactForces* pressure,
                             const JointSet* joints)
    : state_(bodies, pressure, up, restBounceSpeed, joints), friction_(state_), assembly_(state_), coupled_(state_, friction_), slides_(state_, friction_), warm_(state_, friction_), shock_(state_, friction_) {
    // Пары с точками контакта — по высоте самой низкой точки, снизу вверх; при равенстве — по
    // номерам тел, чтобы порядок зависел только от самих чисел.
    struct Ordered {
        Real height;
        const ContactPair* pair;
    };
    std::vector<Ordered> ordered;
    for (const ContactPair& pair : pairs) {
        if (pair.manifold.count == 0) continue;
        Real lowest = std::numeric_limits<Real>::infinity();
        for (int i = 0; i < pair.manifold.count; ++i) lowest = std::min(lowest, up.dot(pair.manifold.points[i].onA));
        ordered.push_back({lowest, &pair});
    }
    std::stable_sort(ordered.begin(), ordered.end(), [](const Ordered& x, const Ordered& y) { return x.height < y.height; });

    for (const Ordered& entry : ordered) {
        const ContactPair& pair = *entry.pair;
        const Body& a = state_.bodies[size_t(pair.a)];
        const Body& b = state_.bodies[size_t(pair.b)];
        const size_t first = state_.rows.size();
        for (int i = 0; i < pair.manifold.count; ++i) {
            const ContactPoint& p = pair.manifold.points[i];
            Row row;
            row.key = {pair.a, pair.b, p.id, pair.partA, pair.partB};
            // Общая точка приложения — середина между точками на двух поверхностях.
            row.point = Real(0.5) * (p.onA + p.onB);
            row.normal = pair.manifold.normal;
            row.separation = p.separation;
            row.restitution = std::max(a.restitution, b.restitution);
            row.normalMass = ContactRows::effectiveMass(a, b, row.point, row.normal);
            row.touching = pair.inContact() && p.separation <= kContactDistance;
            row.startSpeed = row.normal.dot(pointVelocity(b, row.point) - pointVelocity(a, row.point));
            row.roundA = pair.manifold.roundA, row.roundB = pair.manifold.roundB, row.faceOnB = pair.manifold.faceOnB;
            state_.rows.push_back(row);
        }
        state_.pairs.push_back(friction_.frictionOf(pair, first, state_.rows.size()));
    }
    if (state_.joints)
        for (const Joint& joint : *state_.joints)
            for (const Bond& bond : joint.bonds(state_.bodies, state_.bodies)) state_.bonds.push_back(bond);
    state_.bondImpulse.assign(state_.bonds.size(), 0), state_.bondGoal.assign(state_.bonds.size(), 0);
    for (const Bond& bond : state_.bonds) state_.bondStart.push_back(state_.speed(ContactRows::bondLine(bond)));
    state_.setDrift(state_.bodies);
}

void ContactSolver::bondImpulses(std::vector<Vector3>& linear, std::vector<Vector3>& angular) const {
    linear.assign(state_.bodies.size(), {}), angular.assign(state_.bodies.size(), {});
    for (size_t j = 0; j < state_.bonds.size(); ++j) {
        const Bond& bond = state_.bonds[j];
        if (bond.driven || bond.soft) continue;
        if (!state_.bodies[size_t(bond.b)].isStatic()) linear[size_t(bond.b)] += state_.bondImpulse[j] * bond.linear, angular[size_t(bond.b)] += state_.bondImpulse[j] * bond.angularB;
        if (!state_.bodies[size_t(bond.a)].isStatic()) linear[size_t(bond.a)] += -state_.bondImpulse[j] * bond.linear, angular[size_t(bond.a)] += -state_.bondImpulse[j] * bond.angularA;
    }
}

void ContactSolver::solve(const std::vector<Real>& goal) {
    const System s = assembly_.assemble(goal);
    // Скользящие и сорвавшиеся пары обходятся по очереди после точного решения остальных. Круги
    // повторяются, пока условия задачи не выполнены с допуском (residual) — или пока поправки не
    // перестанут убывать: при больших отношениях масс округление не даёт им опуститься ниже
    // своего уровня, и дальше круги ничего не меняют.
    //
    // Мерка сходимости — невязка скорости, а не изменение импульсов. В вырожденной задаче импульсы
    // могут меняться от круга к кругу, не меняя скоростей: у пирамиды из 36 кубов круги шли до
    // правила «не убывают» при скоростях, выполненных с первого круга, а импульс ненагруженной
    // точки рос на одну и ту же величину каждый круг.
    Real best = std::numeric_limits<Real>::infinity();
    int stalled = 0;
    for (state_.iterations = 1; state_.iterations <= kMaxIterations; ++state_.iterations) {
        Real worst = coupled_.solveTogether(s, goal), rubbing = 0;
        for (size_t p = 0; p < s.pairs.size(); ++p)
            if (!state_.stuck[s.pairs[p]]) rubbing = std::max(rubbing, friction_.solveFriction(state_.pairs[s.pairs[p]], goal));
        for (size_t p = s.pairs.size(); p-- > 0;)
            if (!state_.stuck[s.pairs[p]]) rubbing = std::max(rubbing, friction_.solveFriction(state_.pairs[s.pairs[p]], goal));
        if (rubbing < kSolved && coupled_.residual(s, goal) < kSolved) return;
        worst = std::max(worst, rubbing);
        if (worst < kSolved) return;
        if (worst < best / 2) best = worst, stalled = 0;
        else if (++stalled == 5) break;
    }
    // Чередование не сошлось, а последним был обход трения: он мог оставить точки сближающимися.
    // Нормали решаются ещё раз — невыполненным остаётся закон трения, а не непроникание. Одно
    // решение оставляет невязку от добавки kProximal (10⁻⁶ от поправки), повторное её убирает.
    for (int pass = 0; pass < 4; ++pass)
        if (coupled_.solveTogether(s, goal) < kSolved) break;
}

void ContactSolver::solveImpacts() {
    // Отскок определяется скоростью, с которой тела пришли, поэтому цели считаются до решения.
    std::vector<Real> goal(state_.rows.size(), kIdle);
    for (size_t i = 0; i < state_.rows.size(); ++i) {
        if (!state_.rows[i].touching) continue;
        // Отскок, который не вывел бы тело из полосы касания, — не отскок: иначе тело,
        // приходящее в покой, отскакивало бы бесконечно много раз за конечное время.
        const Real approach = state_.rows[i].startSpeed;
        goal[i] = approach < -state_.restBounceSpeed ? -state_.rows[i].restitution * approach : Real(0);
    }
    reset();
    state_.holding = false;
    state_.holdTime = 0;
    // Скорость вдоль запертой свободы — нулевая, вдоль свободы с приводом — заданная.
    for (size_t j = 0; j < state_.bonds.size(); ++j) state_.bondGoal[j] = state_.bonds[j].speed;
    solve(goal);
}

ContactForces ContactSolver::pressures() const {
    ContactForces known;
    for (size_t i = 0; i < state_.rows.size(); ++i)
        if (state_.rows[i].touching) known[state_.rows[i].key] = {state_.sum[i], {}, 0};
    return known;
}

void ContactSolver::reset() {
    state_.sum.assign(state_.rows.size(), 0);
    for (Pair& pair : state_.pairs) pair.impulse = {};
    std::fill(state_.bondImpulse.begin(), state_.bondImpulse.end(), Real(0));
}

void ContactSolver::solveOverlap() {
    std::vector<Real> goal(state_.rows.size(), kIdle);
    for (size_t i = 0; i < state_.rows.size(); ++i)
        if (state_.rows[i].touching) goal[i] = state_.rows[i].separation < -kOverlap / 2 ? -state_.rows[i].separation : Real(0);
    reset();
    state_.holding = false;
    // Разошедшаяся связь за единицу времени возвращается на место; пройденный предел хода — тоже,
    // а не пройденный не даёт к себе подойти ближе, чем осталось.
    for (size_t j = 0; j < state_.bonds.size(); ++j) state_.bondGoal[j] = state_.bonds[j].oneSided ? -state_.bonds[j].gap : -state_.bonds[j].error;
    solve(goal);
}

ContactForces ContactSolver::solveRestingContacts(Real h, const ContactForces& previous, bool shockPropagation) {
    std::vector<Real> goal(state_.rows.size(), kIdle);
    for (size_t i = 0; i < state_.rows.size(); ++i) {
        if (state_.rows[i].touching) goal[i] = -state_.rows[i].startSpeed - h * state_.rows[i].drift;
    }
    reset();
    state_.holding = true;
    state_.holdTime = h;
    // Связь за отрезок не должна разойтись: как и у точки контакта, перемещение равно средней из
    // скоростей в начале и в конце, умноженной на h, плюс уход от вращения тел.
    for (size_t j = 0; j < state_.bonds.size(); ++j) {
        const Bond& bond = state_.bonds[j];
        state_.bondGoal[j] = bond.soft ? -h * bond.stiffness * bond.stretch / (h * h * bond.stiffness + h * bond.damping)
                                 : bond.driven ? bond.speed : -state_.bondStart[j] - h * bond.drift + (j < state_.bondBias.size() ? state_.bondBias[j] : Real(0));
    }
    warm_.apply(goal, previous, h);
    solve(goal);
    if (shockPropagation) shock_.pass(goal);

    // Сила трения пары запоминается поровну на её касающихся точках: при тёплом старте доли
    // складываются обратно.
    ContactForces forces;
    for (const Pair& pair : state_.pairs) {
        int touching = 0;
        for (size_t i = pair.first; i < pair.last; ++i) touching += state_.rows[i].touching;
        const Real share = touching ? 1 / (h * touching) : Real(0);
        for (size_t i = pair.first; i < pair.last; ++i)
            if (state_.rows[i].touching)
                forces[state_.rows[i].key] = {state_.sum[i] / h, share * (pair.impulse[0] * pair.tangent1 + pair.impulse[1] * pair.tangent2), share * pair.impulse[2]};
    }
    return forces;
}

} // namespace kp
