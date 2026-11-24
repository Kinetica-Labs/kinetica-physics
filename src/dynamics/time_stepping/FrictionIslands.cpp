#include <dynamics/time_stepping/FrictionIslands.h>
#include <dynamics/time_stepping/CoupledFriction.h>
#include <algorithm>
#include <numeric>

namespace kp {
namespace {
// Нумеруем только подвижные тела, связанные контактами или сочленениями.
// Рабочие векторы потока переиспользуются на следующем подшаге.
class Components {
public:
    void build(const StepArray<SolverBody>& states, const std::vector<std::pair<int,int>>& edges) {
        ids_.clear();
        for (const auto& [a,b] : edges) for (int id : {a,b})
            if (states[size_t(id)].inverseMass > 0) ids_.push_back(id);
        std::sort(ids_.begin(), ids_.end());
        ids_.erase(std::unique(ids_.begin(), ids_.end()), ids_.end());
        parent_.resize(ids_.size());
        std::iota(parent_.begin(), parent_.end(), 0);
        for (const auto& [a,b] : edges) {
            const int i = root(a), j = root(b);
            if (i >= 0 && j >= 0) parent_[size_t(std::max(i,j))] = std::min(i,j);
        }
    }
    int of(int a, int b) {
        const int i = root(a);
        return i >= 0 ? i : root(b);
    }
    size_t size() const { return ids_.size(); }
private:
    int root(int id) {
        const auto p = std::lower_bound(ids_.begin(), ids_.end(), id);
        if (p == ids_.end() || *p != id) return -1;
        int i = int(p - ids_.begin());
        while (parent_[size_t(i)] != i) i = parent_[size_t(i)] = parent_[size_t(parent_[size_t(i)])];
        return i;
    }
    std::vector<int> ids_, parent_;
};

struct Island {
    size_t contacts[2]{}, count = 0;
    std::vector<size_t> guides;
    bool hasSoft = false;
    bool coupled() const { return count == 2 && !hasSoft; }
    void append(size_t c) { if (count < 2) contacts[count] = c; ++count; }
};

class Partition {
public:
    void build(const StepArray<SoftContact>& contacts, const StepArray<SolverBody>& states,
               const DirectJoints& locked, const StepArray<SoftJoint>& soft) {
        edges_.clear();
        present_.clear();
        for (size_t i = 0; i < contacts.size(); ++i) {
            if (!CoupledFriction::participates(contacts[i])) continue;
            present_.push_back(i);
            const auto row = contacts[i].normalRow(0);
            edges_.emplace_back(row.a, row.b);
        }
        for (const auto& b : locked.rows()) edges_.emplace_back(b.a,b.b);
        for (const auto& s : soft) edges_.emplace_back(s.firstRow().a,s.firstRow().b);
        groups_.build(states, edges_);
        slots_.assign(groups_.size(), -1);
        used_ = 0;
        for (size_t i : present_) {
            const auto row = contacts[i].normalRow(0);
            const int root = groups_.of(row.a,row.b);
            if (root >= 0) ensure(size_t(root)).append(i);
        }
        for (const auto& link : soft)
            if (auto* group = find(link.firstRow())) group->hasSoft = true;
        for (size_t i = 0; i < locked.rows().size(); ++i)
            if (auto* group = find(locked.rows()[i]); group && group->coupled()) group->guides.push_back(i);
    }
    auto begin() const { return islands_.begin(); }
    auto end() const { return islands_.begin() + used_; }

private:
    Island& ensure(size_t root) {
        int& slot = slots_[root];
        if (slot >= 0) return islands_[size_t(slot)];
        slot = int(used_++);
        if (islands_.size() < used_) islands_.emplace_back();
        Island& island = islands_[size_t(slot)];
        island.count = 0;
        island.hasSoft = false;
        island.guides.clear();
        return island;
    }
    Island* find(const Bond& b) {
        const int root = groups_.of(b.a,b.b);
        if (root < 0 || slots_[size_t(root)] < 0) return nullptr;
        return &islands_[size_t(slots_[size_t(root)])];
    }
    Components groups_;
    std::vector<std::pair<int,int>> edges_;
    std::vector<int> slots_;
    std::vector<size_t> present_;
    std::vector<Island> islands_;
    size_t used_ = 0;
};

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
        thread_local Partition islands;
        islands.build(contacts, states, locked, soft);
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
