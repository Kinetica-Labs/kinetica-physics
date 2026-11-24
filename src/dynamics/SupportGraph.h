#pragma once
// Слои опор от неподвижного основания. Порядок меняет обход, а не массу тел.
#include <body/Body.h>
#include <math/StepMemory.h>
#include <algorithm>
#include <utility>

namespace kp {

class SupportGraph {
public:
    SupportGraph(const std::vector<Body>& bodies, Vector3 down)
        : bodies_(bodies), down_(down), head_(bodies.size(), -1), layer_(bodies.size(), -1) {
        for (size_t i = 0; i < bodies.size(); ++i) {
            if (bodies[i].isStatic() && !isDriven(bodies[i])) {
                layer_[i] = 0;
                queue_.push_back(int(i));
            }
        }
    }

    bool hasBase() const { return !queue_.empty(); }

    void add(size_t contact, int a, int b, Vector3 normal) {
        const Real alignment = normal.dot(down_);
        if (alignment == 0 || isDriven(bodies_[a]) || isDriven(bodies_[b])) return;
        const int lower = alignment < 0 ? a : b, upper = alignment < 0 ? b : a;
        if (bodies_[upper].isStatic()) return;
        edges_.push_back({contact, lower, upper, head_[lower]});
        head_[lower] = int(edges_.size()) - 1;
    }

    StepArray<std::pair<int, size_t>> ordered() {
        for (size_t i = 0; i < queue_.size(); ++i) {
            const int lower = queue_[i];
            for (int e = head_[lower]; e >= 0; e = edges_[e].next) {
                const int upper = edges_[e].upper;
                if (layer_[upper] >= 0) continue;
                layer_[upper] = layer_[lower] + 1;
                queue_.push_back(upper);
            }
        }
        StepArray<std::pair<int, size_t>> ranked;
        for (const Edge& edge : edges_)
            if (layer_[edge.lower] >= 0) ranked.emplace_back(layer_[edge.lower], edge.contact);
        std::sort(ranked.begin(), ranked.end()); // при равенстве слоя — в исходном порядке пар
        return ranked;
    }

private:
    struct Edge { size_t contact; int lower, upper, next; };
    const std::vector<Body>& bodies_;
    Vector3 down_;
    StepArray<int> head_, layer_, queue_;
    StepArray<Edge> edges_;
};

template<class Collect, class Visit>
void sweepSupports(const std::vector<Body>& bodies, Vector3 down, Collect collect, Visit visit) {
    if (down.lengthSquare() == 0) return;
    const StepMemory::Scope scratch;
    SupportGraph graph(bodies, down);
    if (!graph.hasBase()) return;
    collect(graph);
    const auto order = graph.ordered();
    for (const auto& entry : order) visit(entry.second, false);
    for (auto it = order.rbegin(); it != order.rend(); ++it) visit(it->second, true);
}

} // namespace kp
