#include <collision/CandidateMemory.h>

namespace kp {

const std::vector<CandidateMemory::Pair>* CandidateMemory::valid(const std::vector<Body>& bodies, const StepArray<Real>& reach) const {
    if (origin_.size() != bodies.size()) return nullptr;
    for (size_t i = 0; i < bodies.size(); ++i) {
        const Body& body = bodies[i];
        const Origin& was = origin_[i];
        if (body.shape.get() != was.shape) return nullptr;
        if (!body.shape) continue;
        // 2 sin(θ/2) — удвоенная длина векторной части кватерниона поворота с прошлой постройки.
        const Quaternion turn = body.orientation * was.orientation.conjugate();
        const Real drift = (body.position - was.position).length() + 2 * Vector3{turn.x, turn.y, turn.z}.length() * body.boundingRadius();
        if (drift + reach[i] - was.reach > kSlack) return nullptr;
    }
    return &candidates_;
}

void CandidateMemory::replace(const std::vector<Body>& bodies, const StepArray<Real>& reach, const StepArray<Pair>& candidates) {
    origin_.resize(bodies.size());
    for (size_t i = 0; i < bodies.size(); ++i) origin_[i] = {bodies[i].position, bodies[i].orientation, bodies[i].shape.get(), reach[i]};
    candidates_.assign(candidates.begin(), candidates.end());
}

} // namespace kp
