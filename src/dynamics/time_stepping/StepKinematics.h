#pragma once

#include <body/Body.h>
#include <dynamics/time_stepping/SolverBody.h>
#include <math/StepMemory.h>

namespace kp::step_motion {

// Момент импульса в мире: L = R I Rᵀ ω, кг·м²/с.
inline Vector3 momentumOf(const Body& body, Vector3 w) {
    const auto inverse = body.mass.inverseInertia;
    const Vector3 inertia{1 / inverse.x, 1 / inverse.y, 1 / inverse.z};
    return body.orientation.rotate(inertia.scaled(body.orientation.rotateInverse(w)));
}

// Тензор инерции вытянутого тела меняется при повороте. Направление шага берём в его середине.
inline void advancePose(Body& body, const SolverBody& state, Real h) {
    body.angularMomentum = momentumOf(body, state.w);
    body.position += h * state.v;
    const auto inverse = body.mass.inverseInertia;
    const bool round = inverse.x == inverse.y && inverse.y == inverse.z;
    const Vector3 turn = round ? h * state.w : h * angularVelocityAt(body, (Quaternion::fromRotationVector((h / 2) * state.w) * body.orientation).unit());
    body.orientation = (Quaternion::fromRotationVector(turn) * body.orientation).unit();
}

// Приложенная сила действует в течение h секунд: Δv = h F / m, Δω = h I⁻¹ τ.
inline void applyForces(StepArray<SolverBody>& states, const std::vector<Body>& bodies, Vector3 gravity, Real h) {
    for (size_t i = 0; i < states.size(); ++i) {
        if (states[i].inverseMass <= 0) continue;
        states[i].v = (1 / (1 + h * bodies[i].linearDamping)) * states[i].v + h * gravity;
        states[i].w = (1 / (1 + h * bodies[i].angularDamping)) * states[i].w;
        if (!isForced(bodies[i])) continue;
        states[i].v += (h * states[i].inverseMass) * bodies[i].force;
        states[i].w += h * states[i].spinFrom(bodies[i].torque);
    }
}

} // namespace kp::step_motion
