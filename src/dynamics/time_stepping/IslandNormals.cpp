#include <dynamics/time_stepping/IslandNormals.h>

#include <dynamics/event_driven/ActiveSet.h>

#include <algorithm>

namespace kp {

namespace {

constexpr int kEntering = 4;                // сколько самых нарушенных точек пары тел входит в набор за круг
constexpr Real kViolated = Real(1e-6);      // м/с: сближение сверх цели, с которого точка считается нарушенной
constexpr char kOutside = 0, kWorking = 1, kEntered = 2; // точка вне рабочего набора, в нём, только что вошла

} // namespace

// Островки — объединением тел, связанных контактами; неподвижное тело островков не связывает.
void IslandNormals::choose(const std::vector<SoftContact>& contacts, const std::vector<Body>& bodies) {
    std::vector<int> root(bodies.size()), count(bodies.size(), 0);
    for (size_t i = 0; i < root.size(); ++i) root[i] = int(i);
    const auto find = [&](int i) {
        while (root[size_t(i)] != i) i = root[size_t(i)] = root[size_t(root[size_t(i)])];
        return i;
    };
    // У контакта хотя бы одно тело подвижно: его и берём представителем.
    const auto bodyOf = [&](const SoftContact& contact) {
        const SoftContact::NormalRow row = contact.normalRow(0);
        return bodies[size_t(row.a)].isStatic() ? row.b : row.a;
    };
    for (const SoftContact& contact : contacts) {
        if (contact.pointCount() == 0) continue;
        const SoftContact::NormalRow row = contact.normalRow(0);
        if (!bodies[size_t(row.a)].isStatic() && !bodies[size_t(row.b)].isStatic()) root[size_t(std::max(find(row.a), find(row.b)))] = std::min(find(row.a), find(row.b));
    }
    for (const SoftContact& contact : contacts)
        if (contact.pointCount() > 0) count[size_t(find(bodyOf(contact)))] += int(contact.pointCount());
    direct_.assign(contacts.size(), 0), rows_.clear();
    for (size_t c = 0; c < contacts.size(); ++c) {
        if (contacts[c].pointCount() == 0 || count[size_t(find(bodyOf(contacts[c])))] > rowLimit_) continue;
        direct_[c] = 1;
        for (size_t k = 0; k < contacts[c].pointCount(); ++k) rows_.push_back({c, k});
    }
    working_.assign(rows_.size(), kOutside);
}

// Скорость вдоль нормали точки сверх цели; отрицательная — условие v_n ≥ цель нарушено.
Real IslandNormals::excess(const SoftContact::NormalRow& row, const std::vector<SolverBody>& states, const Pass& pass) const {
    const SolverBody &a = states[size_t(row.a)], &b = states[size_t(row.b)];
    const Real speed = row.normal.dot(b.v + b.w.cross(row.rB) - a.v - a.w.cross(row.rA));
    const Real goal = row.gap > 0 ? -row.gap / pass.h : pass.withBias ? std::min(-biasRate_ * row.gap, pass.pushSpeed) : Real(0);
    return speed - goal;
}

// Рабочий набор расширяется, пока вне его есть нарушенные точки. На выходе это решение полной
// задачи: у точки вне набора λ = 0 и v_n ≥ цель, у точек набора дополнительность даёт ActiveSet.
void IslandNormals::solve(std::vector<SoftContact>& contacts, const std::vector<Body>& bodies, std::vector<SolverBody>& states, bool withBias, Real h, Real pushSpeed) {
    if (direct_.size() != contacts.size()) choose(contacts, bodies);
    for (size_t c = 0; c < contacts.size(); ++c)
        if (!direct_[c]) contacts[c].solveNormals(states, withBias, h, pushSpeed);
    const Pass pass{withBias, h, pushSpeed};
    for (size_t r = 0; r < rows_.size(); ++r) working_[r] = contacts[rows_[r].contact].normalRow(rows_[r].point).impulse > 0 ? kWorking : kOutside;
    do solveWorking(contacts, bodies, states, pass);
    while (extend(contacts, states, pass));
}

// Неизвестные — полные импульсы точек набора: из скорости вычитается вклад уже накопленных, A λ_прежн.
// Добавка kProximal (ActiveSet) держит ответ у прежнего: у четырёх точек на грани матрица
// вырожденная, и без неё силу между ними можно распределить как угодно.
void IslandNormals::solveWorking(std::vector<SoftContact>& contacts, const std::vector<Body>& bodies, std::vector<SolverBody>& states, const Pass& pass) {
    ContactSystem system;
    std::vector<size_t> listed;
    std::vector<Real> impulse, free;
    std::vector<char> pressing;
    for (size_t r = 0; r < rows_.size(); ++r) {
        if (working_[r] == kOutside) continue;
        const SoftContact::NormalRow row = contacts[rows_[r].contact].normalRow(rows_[r].point);
        system.lines.push_back({row.a, row.b, row.normal, row.rA.cross(row.normal), row.rB.cross(row.normal)});
        listed.push_back(r), impulse.push_back(row.impulse), free.push_back(excess(row, states, pass));
        pressing.push_back(row.impulse > 0 || working_[r] == kEntered), working_[r] = kWorking;
    }
    const size_t total = listed.size();
    if (total == 0) return;
    system.matrix = ContactSystem::responseMatrix(bodies, system.lines);
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
    std::vector<char> noBondSet;
    std::vector<Real> force(total);
    ActiveSet(bodies, system, noBonds, pressing, noBondSet).solve(std::vector<char>(total, 1), free, force);
    for (size_t i = 0; i < total; ++i) contacts[rows_[listed[i]].contact].setNormalImpulse(states, rows_[listed[i]].point, std::max(Real(0), force[i]));
}

// В набор входят самые нарушенные точки вне его — не больше kEntering на пару тел: у составных тел
// на пару приходятся сотни точек, а независимых условий между двумя телами шесть. false — нарушенных нет.
bool IslandNormals::extend(const std::vector<SoftContact>& contacts, const std::vector<SolverBody>& states, const Pass& pass) {
    bool entered = false;
    std::vector<std::pair<Real, size_t>> violated;
    for (size_t first = 0, last = 0; first < rows_.size(); first = last) {
        const SoftContact::NormalRow head = contacts[rows_[first].contact].normalRow(rows_[first].point);
        violated.clear();
        for (; last < rows_.size(); ++last) {
            const SoftContact::NormalRow row = contacts[rows_[last].contact].normalRow(rows_[last].point);
            if (row.a != head.a || row.b != head.b) break;
            if (working_[last] != kOutside) continue;
            const Real over = excess(row, states, pass);
            if (over < -kViolated) violated.push_back({over, last});
        }
        std::sort(violated.begin(), violated.end());
        for (size_t k = 0; k < violated.size() && k < size_t(kEntering); ++k) working_[violated[k].second] = kEntered, entered = true;
    }
    return entered;
}

} // namespace kp
