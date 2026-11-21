#include <math/StepMemory.h>
#include <dynamics/time_stepping/IslandNormals.h>

#include <dynamics/event_driven/ActiveSet.h>

#include <math/WorkerPool.h>

#include <algorithm>

namespace kp {

namespace {

constexpr int kEntering = 4;                // сколько самых нарушенных точек пары тел входит в набор за круг
constexpr Real kViolated = Real(1e-6);      // м/с: сближение сверх цели, с которого точка считается нарушенной
constexpr char kOutside = 0, kWorking = 1, kEntered = 2; // точка вне рабочего набора, в нём, только что вошла
constexpr size_t kParallelRows = 256;       // меньше точек дешевле решить в одном потоке, чем будить остальные

} // namespace

// Островки — объединением тел, связанных контактами; неподвижное тело островков не связывает и
// получает место в каждом островке, которого касается.
void IslandNormals::choose(const StepArray<SoftContact>& contacts, const std::vector<Body>& bodies) {
    StepArray<int> root(bodies.size()), count(bodies.size(), 0), islandOf(bodies.size(), -1), place(bodies.size(), -1);
    for (size_t i = 0; i < root.size(); ++i) root[i] = int(i);
    const auto find = [&](int i) {
        while (root[size_t(i)] != i) i = root[size_t(i)] = root[size_t(root[size_t(i)])];
        return i;
    };
    const auto top = [&](const SoftContact::NormalRow& row) { return find(bodies[size_t(row.a)].isStatic() ? row.b : row.a); };
    for (const SoftContact& contact : contacts) {
        const SoftContact::NormalRow row = contact.normalRow(0);
        if (!bodies[size_t(row.a)].isStatic() && !bodies[size_t(row.b)].isStatic()) root[size_t(std::max(find(row.a), find(row.b)))] = std::min(find(row.a), find(row.b));
    }
    for (const SoftContact& contact : contacts) count[size_t(top(contact.normalRow(0)))] += int(contact.pointCount());
    const auto seat = [&](Island& island, int body) {
        const Body& source = bodies[size_t(body)];
        if (!source.isStatic() && place[size_t(body)] >= 0) return place[size_t(body)];
        for (size_t k = 0; source.isStatic() && k < island.members.size(); ++k)
            if (island.members[k] == body) return int(k);
        Body slim;
        slim.mass = source.mass, slim.orientation = source.orientation;
        island.members.push_back(body), island.bodies.push_back(slim);
        return source.isStatic() ? int(island.members.size()) - 1 : place[size_t(body)] = int(island.members.size()) - 1;
    };
    direct_.assign(contacts.size(), 0), islands_.clear();
    for (size_t c = 0; c < contacts.size(); ++c) {
        const SoftContact::NormalRow row = contacts[c].normalRow(0);
        const size_t leader = size_t(top(row));
        if (count[leader] > rowLimit_) continue;
        if (islandOf[leader] < 0) islandOf[leader] = int(islands_.size()), islands_.emplace_back();
        Island& island = islands_[size_t(islandOf[leader])];
        const int a = seat(island, row.a), b = seat(island, row.b);
        for (size_t k = 0; k < contacts[c].pointCount(); ++k) island.rows.push_back({c, k, a, b});
        direct_[c] = 1;
    }
    for (Island& island : islands_) island.working.assign(island.rows.size(), kOutside);
}

// Скорость вдоль нормали точки сверх цели; отрицательная — условие v_n ≥ цель нарушено.
Real IslandNormals::excess(const SoftContact::NormalRow& row, const StepArray<SolverBody>& states, const Pass& pass) const {
    const SolverBody &a = states[size_t(row.a)], &b = states[size_t(row.b)];
    const Real speed = row.normal.dot(b.v + b.w.cross(row.rB) - a.v - a.w.cross(row.rA));
    const Real goal = row.gap > 0 ? -row.gap / pass.h : pass.withBias ? std::min(-biasRate_ * row.gap, pass.pushSpeed) : Real(0);
    return speed - goal;
}

// Островки не связаны: каждый пишет только в свои контакты и свои подвижные тела, поэтому решаются
// заданиями пула потоков, и ответ от числа потоков не зависит.
void IslandNormals::solve(StepArray<SoftContact>& contacts, const std::vector<Body>& bodies, StepArray<SolverBody>& states, bool withBias, Real h, Real pushSpeed,
                          const StepArray<char>* moving) {
    if (direct_.size() != contacts.size()) choose(contacts, bodies);
    const auto stirs = [&](int body) { return !moving || (*moving)[size_t(body)]; };
    levels.each(contacts, states, [&](size_t c) {
        if (!direct_[c] && (stirs(contacts[c].normalRow(0).a) || stirs(contacts[c].normalRow(0).b))) contacts[c].solveNormals(states, withBias, h, pushSpeed);
    });
    const Pass pass{withBias, h, pushSpeed};
    size_t rows = 0;
    for (const Island& island : islands_) rows += island.rows.size();
    const auto one = [&](size_t k) {
        if (std::any_of(islands_[k].members.begin(), islands_[k].members.end(), stirs)) solveIsland(islands_[k], contacts, bodies, states, pass);
    };
    if (rows < kParallelRows) for (size_t k = 0; k < islands_.size(); ++k) one(k);
    else WorkerPool::shared().run(islands_.size(), one);
}

// Рабочий набор расширяется, пока вне его есть нарушенные точки. На выходе это решение полной
// задачи: у точки вне набора λ = 0 и v_n ≥ цель, у точек набора дополнительность даёт ActiveSet.
void IslandNormals::solveIsland(Island& island, StepArray<SoftContact>& contacts, const std::vector<Body>& bodies, StepArray<SolverBody>& states, const Pass& pass) const {
    for (size_t k = 0; k < island.members.size(); ++k) island.bodies[k].orientation = bodies[size_t(island.members[k])].orientation;
    for (size_t r = 0; r < island.rows.size(); ++r) island.working[r] = contacts[island.rows[r].contact].normalRow(island.rows[r].point).impulse > 0 ? kWorking : kOutside;
    // Предел — на всё решение островка: каждому расширению набора достаётся его остаток.
    int spent = 0;
    do spent += solveWorking(island, contacts, states, pass, std::max(0, iterations_ - spent));
    while ((iterations_ == 0 || spent < iterations_) && extend(island, contacts, states, pass));
}

// Неизвестные — полные импульсы точек набора: из скорости вычитается вклад уже накопленных, A λ_прежн.
// Добавка kProximal (ActiveSet) держит ответ у прежнего: у четырёх точек на грани матрица
// вырожденная, и без неё силу между ними можно распределить как угодно.
int IslandNormals::solveWorking(Island& island, StepArray<SoftContact>& contacts, StepArray<SolverBody>& states, const Pass& pass, int allowed) const {
    ContactSystem system;
    StepArray<size_t> listed;
    StepArray<Real> impulse, free;
    StepArray<char> pressing;
    for (size_t r = 0; r < island.rows.size(); ++r) {
        if (island.working[r] == kOutside) continue;
        const Row& at = island.rows[r];
        const SoftContact::NormalRow row = contacts[at.contact].normalRow(at.point);
        system.lines.push_back({at.a, at.b, row.normal, row.rA.cross(row.normal), row.rB.cross(row.normal)});
        listed.push_back(r), impulse.push_back(row.impulse), free.push_back(excess(row, states, pass));
        pressing.push_back(row.impulse > 0 || island.working[r] == kEntered), island.working[r] = kWorking;
    }
    const size_t total = listed.size();
    if (total == 0) return 0;
    system.matrix = ContactSystem::responseMatrix(island.bodies, system.lines);
    system.diagonal.assign(total, 0), system.soft.assign(total, 0);
    for (size_t i = 0; i < total; ++i) {
        system.rows.push_back(i), system.order.push_back(i);
        for (const ContactSystem::Entry& entry : system.matrix[i]) {
            if (entry.column == i) system.diagonal[i] = entry.value;
            free[i] -= entry.value * impulse[entry.column];
        }
        free[i] -= kProximal * system.diagonal[i] * impulse[i];
    }
    const std::vector<Bond> noBonds;
    StepArray<char> noBondSet;
    StepArray<Real> force(total);
    ActiveSet set(island.bodies, system, noBonds, pressing, noBondSet);
    set.iterations = allowed, set.start = &impulse;
    set.solve(StepArray<char>(total, 1), free, force);
    for (size_t i = 0; i < total; ++i) contacts[island.rows[listed[i]].contact].setNormalImpulse(states, island.rows[listed[i]].point, std::max(Real(0), force[i]));
    return set.spent();
}

// В набор входят самые нарушенные точки вне его — не больше kEntering на пару тел: у составных тел
// на пару приходятся сотни точек, а независимых условий между двумя телами шесть. false — нарушенных нет.
bool IslandNormals::extend(Island& island, const StepArray<SoftContact>& contacts, const StepArray<SolverBody>& states, const Pass& pass) const {
    bool entered = false;
    StepArray<std::pair<Real, size_t>> violated;
    for (size_t first = 0, last = 0; first < island.rows.size(); first = last) {
        violated.clear();
        for (; last < island.rows.size(); ++last) {
            const Row& at = island.rows[last];
            if (at.a != island.rows[first].a || at.b != island.rows[first].b) break;
            if (island.working[last] != kOutside) continue;
            const Real over = excess(contacts[at.contact].normalRow(at.point), states, pass);
            if (over < -kViolated) violated.push_back({over, last});
        }
        std::sort(violated.begin(), violated.end());
        for (size_t k = 0; k < violated.size() && k < size_t(kEntering); ++k) island.working[violated[k].second] = kEntered, entered = true;
    }
    return entered;
}

} // namespace kp
