#include <math/StepMemory.h>
#include <dynamics/time_stepping/MissedContacts.h>

#include <algorithm>

namespace kp {

MissedContacts::MissedContacts(const std::vector<ContactPair>& pairs, const std::vector<Body>& bodies, StepArray<size_t> owner, Real depth, Real boundary)
    : owner_(std::move(owner)), depth_(depth), boundary_(boundary) {
    for (const ContactPair& pair : pairs) adopt(pair, bodies, false);
}

void MissedContacts::adopt(const ContactPair& pair, const std::vector<Body>& bodies, bool withContact) {
    if (withContact) owner_.push_back(pairs_.size());
    pairs_.push_back(pair);
    radiusA_.push_back(bodies[size_t(pair.a)].part(pair.partA).boundingRadius());
    radiusB_.push_back(bodies[size_t(pair.b)].part(pair.partB).boundingRadius());
}

void MissedContacts::note(const StepArray<SoftContact>& contacts) {
    known_.assign(pairs_.size(), 0);
    for (size_t c = 0; c < contacts.size(); ++c) known_[owner_[c]] = std::max(known_[owner_[c]], contacts[c].deepest());
}

// Полная проверка пары дорога, поэтому до неё три дешёвых отсева — каждый доказывает, что глубина
// вырасти на допуск не могла:
//   — точка тела движется не быстрее |v| + |ω| R, и за подшаг точки пары сближаются не больше чем на
//     (|v_a| + |ω_a| R_a + |v_b| + |ω_b| R_b) h;
//   — с начала обновления точка тела ушла не дальше |сдвиг| + 2 |sin(θ/2)| R (хорда поворота), и
//     пара, до которой было дальше, ещё не коснулась;
//   — зазор вдоль любого направления не больше расстояния со знаком
//     (proofs/plane_gap_bounds_distance.smt2): если вдоль прежней нормали пары тела не вошли глубже
//     известного, то и вообще не вошли.
bool MissedContacts::find(StepArray<SoftContact>& contacts, const std::vector<Body>& bodies, const StepArray<SolverBody>& states, Real h, const Maker& make) {
    StepArray<Real> speed(states.size()), spin(states.size()), shift(states.size()), chord(states.size());
    for (size_t i = 0; i < states.size(); ++i) {
        const Quaternion& q = states[i].turned;
        speed[i] = states[i].v.length(), spin[i] = states[i].w.length(), shift[i] = states[i].moved.length(), chord[i] = 2 * Vector3{q.x, q.y, q.z}.length();
    }
    bool found = false;
    for (size_t p = 0; p < pairs_.size(); ++p) {
        const ContactPair& pair = pairs_[p];
        const size_t a = size_t(pair.a), b = size_t(pair.b);
        if ((speed[a] + spin[a] * radiusA_[p] + speed[b] + spin[b] * radiusB_[p]) * h <= depth_) continue;
        if (pair.distance - (shift[a] + chord[a] * radiusA_[p] + shift[b] + chord[b] * radiusB_[p]) >= -depth_) continue;
        const Body partA = bodies[a].part(pair.partA), partB = bodies[b].part(pair.partB);
        if (pair.manifold.count > 0 && gapAlong(partA, partB, pair.manifold.normal) >= -(known_[p] + depth_)) continue;
        std::optional<ContactPair> now = examinePair(partA, partB, Vector3{}, 0, boundary_);
        if (!now) continue;
        Real deepest = 0;
        for (int k = 0; k < now->manifold.count; ++k) deepest = std::max(deepest, -now->manifold.points[size_t(k)].separation);
        if (deepest <= known_[p] + depth_) continue;
        now->a = pair.a, now->b = pair.b, now->partA = pair.partA, now->partB = pair.partB;
        contacts.push_back(make(*now)), owner_.push_back(p), found = true;
    }
    return found;
}

} // namespace kp
