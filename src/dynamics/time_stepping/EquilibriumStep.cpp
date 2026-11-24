#include "EquilibriumStep.h"
#include "StepKinematics.h"

#include <algorithm>
#include <array>

namespace kp {
namespace {

constexpr Real kLinearSpeed = Real(1e-8);   // м/с
constexpr Real kAngularSpeed = Real(1e-8);  // рад/с
constexpr Real kTravel = Real(1e-10);       // м за подшаг

bool atRest(const std::vector<Body>& bodies, const StepArray<SolverBody>& states, Real h) {
    for (size_t i = 0; i < bodies.size(); ++i) {
        const Body& body = bodies[i];
        if (isDriven(body)) return false;
        if (body.isStatic()) continue;
        const Real spin = states[i].w.length();
        const Real radius = body.shape ? body.boundingRadius() : Real(0);
        const Real speed = states[i].v.length() + radius * spin;
        if (!(spin <= kAngularSpeed && speed <= kLinearSpeed && h * speed <= kTravel)) return false;
    }
    return true;
}

bool carryGuides(const JointSet& joints, const std::vector<Body>& bodies, StepArray<SolverBody>& trial,
                 const std::vector<std::vector<Real>>& memory, Real h) {
    if (memory.size() != joints.size()) return false;
    std::vector<Bond> rows;
    for (size_t j = 0; j < joints.size(); ++j) {
        rows.reserve(memory[j].size());
        joints[j].writeBonds(bodies, bodies, rows);
        if (rows.size() != memory[j].size()) return false;
        for (size_t k = 0; k < rows.size(); ++k) {
            const Bond& row = rows[k];
            const Real speedLimit = row.linear.lengthSquare() == 0 ? kAngularSpeed : kLinearSpeed;
            if (row.soft || row.driven || row.oneSided || !(abs(row.error) <= h * speedLimit)) return false;
            const Real impulse = memory[j][k];
            if (impulse == 0) continue;
            auto& a = trial[size_t(row.a)];
            auto& b = trial[size_t(row.b)];
            if (a.inverseMass != 0) {
                a.v -= (a.inverseMass * impulse) * row.linear;
                a.w -= a.spinFrom(impulse * row.angularA);
            }
            if (b.inverseMass != 0) {
                b.v += (b.inverseMass * impulse) * row.linear;
                b.w += b.spinFrom(impulse * row.angularB);
            }
        }
    }
    return true;
}

struct FrictionShare {
    Vector3 tangent;
    Real twist = 0;
};

struct SupportPatch {
    std::array<Real, 4> normalForce{};
    std::array<Vector3, 4> at{};
    Real load = 0, moment = 0;
    Vector3 center{}, tangent{};
};

// Прежняя сила N в той же точке даёт нормальный импульс h N. Все точки должны ещё касаться.
bool readSupport(const ContactPair& pair, const ContactForces& forces, Real h, SupportPatch& patch) {
    const auto& m = pair.manifold;
    for (int k = 0; k < m.count; ++k) {
        const auto& point = m.points[size_t(k)];
        if (!(abs(point.separation) <= h * kLinearSpeed)) return false;
        for (int j = 0; j < k; ++j)
            if (m.points[size_t(j)].id == point.id) return false;
        const auto* entry = forces.find({pair.a, pair.b, point.id, pair.partA, pair.partB});
        if (entry == forces.end() || !(entry->second.normal >= 0)) return false;
        const ContactForce& force = entry->second;
        patch.at[size_t(k)] = Real(0.5) * (point.onA + point.onB);
        patch.center += patch.at[size_t(k)];
        patch.normalForce[size_t(k)] = force.normal;
        patch.load += force.normal;
        patch.moment += force.twist;
        patch.tangent += force.tangent;
    }
    patch.center = (Real(1) / m.count) * patch.center;
    return patch.load > 0;
}

bool carrySupport(const ContactPair& pair, const std::vector<Body>& bodies, StepArray<SolverBody>& trial,
                  const ContactForces& forces, Real h, FrictionShare& next) {
    const auto& m = pair.manifold;
    const auto& a = bodies[size_t(pair.a)];
    const auto& b = bodies[size_t(pair.b)];
    if (m.count < 2 || m.count > 4 || m.roundA != 0 || m.roundB != 0 || a.restitution != 0 || b.restitution != 0 ||
        !a.shape || !b.shape || !a.shape->hull() || !b.shape->hull() || a.parts || b.parts) return false;
    SupportPatch patch;
    if (!readSupport(pair, forces, h, patch)) return false;
    auto& sa = trial[size_t(pair.a)];
    auto& sb = trial[size_t(pair.b)];
    Real leverLoad = 0;
    for (int k = 0; k < m.count; ++k) {
        const Real load = patch.normalForce[size_t(k)];
        const Vector3 at = patch.at[size_t(k)];
        leverLoad += (at - patch.center).length() * load;
        if (load != 0) SolverBody::push(sa, sb, at - a.position, at - b.position, (h * load) * m.normal);
    }
    // В новой касательной плоскости |Fₜ| ≤ μ ΣN, |M| ≤ μ Σ(N r).
    if (!(a.friction >= 0 && b.friction >= 0)) return false;
    const Real mu = sqrt(a.friction * b.friction);
    patch.tangent -= patch.tangent.dot(m.normal) * m.normal;
    const Real length = patch.tangent.length(), limit = mu * patch.load;
    if (!(limit >= 0 && length >= 0)) return false;
    if (length > limit) patch.tangent = (limit / length) * patch.tangent;
    const Real moment = std::clamp(patch.moment, -mu * leverLoad, mu * leverLoad);
    next = {(Real(1) / m.count) * patch.tangent, moment / m.count};
    SolverBody::push(sa, sb, patch.center - a.position, patch.center - b.position, h * patch.tangent);
    if (moment != 0) SolverBody::twist(sa, sb, (h * moment) * m.normal);
    return true;
}

void commitMotion(std::vector<Body>& bodies, const StepArray<SolverBody>& trial, Real h) {
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (trial[i].inverseMass == 0) continue;
        bodies[i].linearMomentum = (1 / trial[i].inverseMass) * trial[i].v;
        step_motion::advancePose(bodies[i], trial[i], h);
    }
}

} // namespace

bool advanceIfBalanced(const SoftStep& step, std::vector<Body>& bodies, const std::vector<ContactPair>& pairs,
                       const JointSet& joints, Vector3 gravity, Real h, ContactForces& forces,
                       const std::vector<std::vector<Real>>& jointMemory, const StepArray<SolverBody>& initial) {
    if (!(h > 0) || step.substeps != 1 || !step.directJoints || !step.coupledFriction || step.rollingResistance != 0 ||
        pairs.empty() || forces.begin() == forces.end() || !atRest(bodies, initial, h)) return false;
    size_t points = 0;
    for (const auto& pair : pairs) points += size_t(pair.manifold.count);
    if (step.directContactRows < 0 || points > size_t(step.directContactRows) || points != size_t(forces.end() - forces.begin())) return false;
    StepArray<SolverBody> trial = initial;
    step_motion::applyForces(trial, bodies, gravity, h);
    if (!carryGuides(joints, bodies, trial, jointMemory, h)) return false;
    StepArray<FrictionShare> reactions(pairs.size());
    for (size_t i = 0; i < pairs.size(); ++i)
        if (!carrySupport(pairs[i], bodies, trial, forces, h, reactions[i])) return false;
    if (!atRest(bodies, trial, h)) return false;

    for (StepStage stage : {StepStage::Contacts, StepStage::WarmStart, StepStage::Joints, StepStage::Normals, StepStage::Recheck})
        notify(step.observer, stage);
    commitMotion(bodies, trial, h);
    notify(step.observer, StepStage::Motion);
    for (size_t i = 0; i < pairs.size(); ++i)
        forces.reviseFriction({pairs[i].a, pairs[i].b, 0, pairs[i].partA, pairs[i].partB}, reactions[i].tangent, reactions[i].twist);
    for (StepStage stage : {StepStage::Recheck, StepStage::Joints, StepStage::Normals, StepStage::Friction, StepStage::Memory})
        notify(step.observer, stage);
    return true;
}

} // namespace kp
