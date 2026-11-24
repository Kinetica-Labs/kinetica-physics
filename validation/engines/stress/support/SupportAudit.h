#pragma once
// Диагностика опыта, выключена при измерении времени. Проверяет фактические
// изменения скоростей до/после каждой добавочной поправки, включая пробные сдвиги.
#include "SupportSweep.h"
#include <cstdio>
#include <cstdlib>

namespace kp::support_study {

struct Totals {
    Vector3 momentum, angular;
    Real kinetic = 0;
    void add(const Body& body, const SolverBody& state) {
        if (body.isStatic()) return;
        const Vector3 p = (1 / body.mass.inverseMass) * state.v;
        const auto omega = body.orientation.rotateInverse(state.w);
        const auto inverse = body.mass.inverseInertia;
        const auto spin = body.orientation.rotate({omega.x/inverse.x, omega.y/inverse.y, omega.z/inverse.z});
        momentum += p;
        angular += body.position.cross(p) + spin;
        kinetic += (p.dot(state.v) + spin.dot(state.w)) / 2;
    }
};

struct Audit {
    unsigned long long fixed = 0, internal = 0;
    Real linearError = 0, angularError = 0, addedEnergy = 0;
    void record(const Totals& before, const Totals& after, Vector3 external, Vector3 torque) {
        linearError = std::max(linearError, (after.momentum-before.momentum-external).length());
        angularError = std::max(angularError, (after.angular-before.angular-torque).length());
        addedEnergy = std::max(addedEnergy, after.kinetic-before.kinetic);
    }
    ~Audit() {
        std::fprintf(stderr,"external,internal,max_deltaP_Ns,max_deltaL_Nms,max_positive_deltaT_J\n%llu,%llu,%.12g,%.12g,%.12g\n",
                     fixed, internal, linearError, angularError, addedEnergy);
    }
};

inline void studyPass(StepArray<SoftContact>& contacts, const std::vector<Body>& bodies,
                      StepArray<SolverBody>& states, Vector3 gravity, Real h) {
    static const char* mode = std::getenv("KP_STUDY_SUPPORT");
    if (!mode || mode[0] == '0' || contacts.empty()) return;
    const bool reversePass = mode[0] != '1';
    const Real allowance = mode[0] == '3' ? kContactDistance : Real(0);
    const bool islands = mode[0] == '4';
    static const bool measure = std::getenv("KP_STUDY_BALANCE") != nullptr;
    if (!measure) {
        sweep(contacts, bodies, states, gravity, h, reversePass, [](const auto&, Real, auto push) { push(); },allowance,islands);
        return;
    }
    static Audit audit;
    const auto check = [&](const SoftContact::NormalRow& row, Real j, auto push) {
        const auto& a = bodies[row.a];
        const auto& b = bodies[row.b];
        const auto sum = [&] {
            Totals totals;
            totals.add(a, states[row.a]);
            totals.add(b, states[row.b]);
            return totals;
        };
        const Totals before = sum();
        push();
        Vector3 external, torque;
        if (a.isStatic() || b.isStatic()) {
            external = (a.isStatic() ? j : -j) * row.normal;
            const Vector3 point = a.isStatic() ? b.position + row.rB : a.position + row.rA;
            torque = point.cross(external);
            ++audit.fixed;
        } else ++audit.internal;
        audit.record(before, sum(), external, torque);
    };
    sweep(contacts, bodies, states, gravity, h, reversePass, check,allowance,islands);
}

} // namespace kp::support_study
