#pragma once
// Опытный проход по опорам. Заимствован порядок распространения снизу вверх,
// массы и моменты инерции обеих сторон всегда настоящие.
#include <dynamics/time_stepping/SoftContact.h>
#include <algorithm>

namespace kp::support_study {

struct Edge { size_t contact; int upper, lower, next; };

// Для нормали от A к B реакция +J n на B поддерживает B, если n · g < 0.
// Исток графа — неподвижная опора. Свободная группа тел не получает слоя.
inline StepArray<size_t> orderOf(const StepArray<SoftContact>& contacts, const std::vector<Body>& bodies, Vector3 gravity) {
    StepArray<int> head(bodies.size(), -1), layer(bodies.size(), -1), queue;
    StepArray<Edge> edges;
    for (size_t i = 0; i < bodies.size(); ++i)
        if (bodies[i].isStatic() && !isDriven(bodies[i])) layer[i] = 0, queue.push_back(int(i));
    if (queue.empty() || gravity.lengthSquare() == 0) return {};
    for (size_t i = 0; i < contacts.size(); ++i) {
        const SoftContact& c = contacts[i];
        const auto r = c.normalRow(0);
        const Real alignment = r.normal.dot(gravity);
        if (alignment == 0 || isDriven(bodies[r.a]) || isDriven(bodies[r.b])) continue;
        bool near = false;
        for (size_t k = 0; k < c.pointCount(); ++k) near |= c.normalRow(k).gap <= kContactDistance;
        if (!near) continue;
        const int lower = alignment < 0 ? r.a : r.b, upper = alignment < 0 ? r.b : r.a;
        if (bodies[upper].isStatic()) continue;
        edges.push_back({i, upper, lower, head[lower]});
        head[lower] = int(edges.size()) - 1;
    }
    for (size_t i = 0; i < queue.size(); ++i) {
        const int from = queue[i];
        for (int e = head[from]; e >= 0; e = edges[e].next) {
            const int to = edges[e].upper;
            if (layer[to] >= 0) continue;
            layer[to] = layer[from] + 1;
            queue.push_back(to);
        }
    }
    StepArray<std::pair<int,size_t>> ranked;
    for (const Edge& edge : edges)
        if (layer[edge.lower] >= 0) ranked.emplace_back(layer[edge.lower], edge.contact);
    std::sort(ranked.begin(), ranked.end());
    StepArray<size_t> order;
    order.reserve(ranked.size());
    for (const auto& item : ranked) order.push_back(item.second);
    return order;
}

// При положительном зазоре g разрешено сближение до -g/h. В закрытом контакте
// цель равна нулю. j = (цель - u)/K, Н·с. Для неподвижной опоры либо двух
// подвижных тел  ΔT = j u + K j²/2 ≤ 0, поскольку u < цель ≤ 0.
inline Real closingExcess(const SoftContact::NormalRow& row, const StepArray<SolverBody>& states, Real h) {
    const auto& a = states[row.a];
    const auto& b = states[row.b];
    const Real speed = row.normal.dot(b.v + b.w.cross(row.rB) - a.v - a.w.cross(row.rA));
    const Real target = -std::max(row.gap, Real(0)) / h;
    return target - speed;
}

template<class Observe>
void project(SoftContact& c, size_t k, StepArray<SolverBody>& states, Real h, Observe&& apply, Real allowedClosure = 0) {
    const auto row = c.normalRow(k);
    const Real excess = closingExcess(row, states, h);
    // Малую невязку покоя уже несёт основной решатель. Вмешиваемся, если за h
    // появится дополнительное перекрытие больше геометрической полосы, м.
    if (excess * h <= allowedClosure) return;
    const Real response = SolverBody::inverseMassAlong(states[row.a], states[row.b], row.rA, row.rB, row.normal);
    if (response <= 0) return;
    const Real next = row.impulse + excess / response;
    apply(row, next - row.impulse, [&] { c.setNormalImpulse(states, k, next); });
}

// Острова соединяются только через подвижные тела. Две груды на одном полу
// имеют независимые реакции: удар на другом конце пола не меняет покоя груды.
inline void keepClosingIslands(StepArray<size_t>& order, const StepArray<SoftContact>& contacts,
                               const StepArray<SolverBody>& states, Real h) {
    StepArray<int> parent(states.size());
    for (size_t i=0; i<parent.size(); ++i) parent[i]=int(i);
    const auto root = [&](int i) {
        while (parent[i]!=i) i=parent[i]=parent[parent[i]];
        return i;
    };
    for (size_t i : order) {
        const auto r=contacts[i].normalRow(0);
        if (states[r.a].inverseMass && states[r.b].inverseMass) {
            const int a=root(r.a), b=root(r.b);
            parent[std::max(a,b)]=std::min(a,b);
        }
    }
    StepArray<char> threatened(states.size(),0);
    const auto component=[&](const SoftContact& c) {
        const auto r=c.normalRow(0);
        return root(states[r.a].inverseMass ? r.a : r.b);
    };
    for (size_t i : order) {
        const auto& c=contacts[i];
        for(size_t k=0; k<c.pointCount(); ++k)
            if (h*closingExcess(c.normalRow(k),states,h)>kContactDistance) { threatened[component(c)]=1; break; }
    }
    order.erase(std::remove_if(order.begin(),order.end(),[&](size_t i){return !threatened[component(contacts[i])];}),order.end());
}

template<class Observe>
void sweep(StepArray<SoftContact>& contacts, const std::vector<Body>& bodies, StepArray<SolverBody>& states,
           Vector3 gravity, Real h, bool reversePass, Observe&& apply, Real allowance=0, bool islands=false) {
    auto order = orderOf(contacts, bodies, gravity);
    if (islands && !order.empty()) keepClosingIslands(order,contacts,states,h);
    for (size_t i : order)
        for (size_t k = 0; k < contacts[i].pointCount(); ++k) project(contacts[i], k, states, h, apply, allowance);
    if (reversePass)
        for (size_t i = order.size(); i > 0; --i) {
            auto& contact = contacts[order[i-1]];
            for (size_t k = contact.pointCount(); k > 0; --k) project(contact, k-1, states, h, apply, allowance);
        }
}

} // namespace kp::support_study
