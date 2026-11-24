#include <dynamics/time_stepping/FrictionIslands.h>
#include <dynamics/time_stepping/CoupledFriction.h>
#include <algorithm>

namespace kp {
namespace {
class Components {
public:
    explicit Components(const StepArray<SolverBody>& states) : parent_(states.size(), -1) {
        for (size_t i = 0; i < states.size(); ++i)
            if (states[i].inverseMass > 0) parent_[i] = int(i);
    }
    int of(int a, int b) {
        if (parent_[size_t(a)] >= 0) return find(a);
        if (parent_[size_t(b)] >= 0) return find(b);
        return -1; // связь двух неподвижных тел не принадлежит динамическому острову
    }
    void join(int a, int b) {
        if (parent_[size_t(a)] < 0 || parent_[size_t(b)] < 0) return;
        a = find(a), b = find(b);
        parent_[size_t(std::max(a,b))] = std::min(a,b);
    }
private:
    int find(int a) {
        while (parent_[size_t(a)] != a) a = parent_[size_t(a)] = parent_[size_t(parent_[size_t(a)])];
        return a;
    }
    StepArray<int> parent_;
};

struct Island {
    size_t contacts[2]{}, count = 0;
    StepArray<size_t> guides;
    bool hasSoft = false;
    bool coupled() const { return count == 2 && !hasSoft; }
    void append(size_t c) { if (count < 2) contacts[count] = c; ++count; }
};

StepArray<Island> partition(const StepArray<SoftContact>& contacts, const StepArray<SolverBody>& states,
                            const DirectJoints& locked, const StepArray<SoftJoint>& soft) {
    Components groups(states);
    for (const auto& c : contacts) { const auto r = c.normalRow(0); groups.join(r.a, r.b); }
    for (const auto& b : locked.rows()) groups.join(b.a, b.b);
    for (const auto& link : soft) { const auto& b = link.firstRow(); groups.join(b.a, b.b); }
    StepArray<Island> islands(states.size());
    const auto islandFor = [&](const auto& row) -> Island* {
        const int root = groups.of(row.a, row.b);
        return root >= 0 ? &islands[size_t(root)] : nullptr;
    };
    for (size_t i = 0; i < contacts.size(); ++i) {
        if (Island* group = islandFor(contacts[i].normalRow(0))) group->append(i);
    }
    for (const auto& link : soft) {
        if (Island* group = islandFor(link.firstRow())) group->hasSoft = true;
    }
    for (size_t i = 0; i < locked.rows().size(); ++i) {
        Island* group = islandFor(locked.rows()[i]);
        if (group && group->coupled()) group->guides.push_back(i);
    }
    return islands;
}

// Направляющие острова перечислены в порядке мировой матрицы. Если здесь все её строки,
// готовое разложение уже подходит. Иначе собираем только строки этого острова.
const DirectJoints& islandGuides(const Island& island, const DirectJoints& locked,
                                const StepArray<SolverBody>& states, DirectJoints& subset) {
    if (island.guides.size() == locked.rows().size()) return locked;
    subset.clear();
    for (size_t k : island.guides) subset.add(locked.rows()[k], locked.accumulated(k));
    subset.assemble(states);
    return subset;
}
}

void solveFrictionIslands(StepArray<SoftContact>& contacts, StepArray<SolverBody>& states, const DirectJoints& locked,
                          const StepArray<SoftJoint>& soft, ContactLevels& levels, Real h, Real rollingArm) {
    StepArray<char> solved;
    if (contacts.size() >= 2) {
        const auto islands = partition(contacts, states, locked, soft);
        DirectJoints subset;
        for (const Island& island : islands) {
            if (!island.coupled()) continue;
            CoupledFriction pair(contacts[island.contacts[0]], contacts[island.contacts[1]]);
            if (!pair.solve(states, islandGuides(island, locked, states, subset), h, rollingArm)) continue;
            if (solved.empty()) solved.assign(contacts.size(), 0);
            solved[island.contacts[0]] = solved[island.contacts[1]] = 1;
        }
    }
    levels.each(contacts, states, [&](size_t c) {
        if (solved.empty() || !solved[c]) contacts[c].solveFriction(states, rollingArm);
    });
}
}
