#include "Check.h"
#include "../examples/scenes/RackDrive.h"
#include "../examples/scenes/RigidRod.h"
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace kp;
namespace {
Vector3 totalAngular(const Body& a, const Body& b) {
    return a.position.cross(a.linearMomentum) + a.angularMomentum + b.position.cross(b.linearMomentum) + b.angularMomentum;
}

void rotatingRod(bool fast) {
    World w = rigid_rod::build();
    w.realtime = fast;
    const Body& a = w.body(BodyId(0));
    const Body& b = w.body(BodyId(1));
    const Vector3 p0 = a.linearMomentum + b.linearMomentum, l0 = totalAngular(a, b);
    Real gap = 0, momentum = 0, angular = 0;
    for (int i = 0; i < 120; ++i) {
        w.step(Real(1) / 120);
        CHECK(!w.report().stalled && w.report().unfinished == 0, "rigid rod did not finish frame %d", i+1);
        gap = std::max(gap, std::abs((b.position - a.position).length() - rigid_rod::length));
        momentum = std::max(momentum, (a.linearMomentum + b.linearMomentum - p0).length());
        angular = std::max(angular, (totalAngular(a, b) - l0).length());
    }
    std::printf("rod,fast=%d,max_length_m=%.12g,max_P_Ns=%.12g,max_L_Nms=%.12g\n", int(fast), gap, momentum, angular);
    CHECK_NEAR(gap, 0, 5e-4);
    CHECK_NEAR(momentum, 0, 1e-9);
    CHECK_NEAR(angular, 0, 1e-9);
}

void reversingRack(bool fast) {
    World w = rack_drive::build();
    w.realtime = fast;
    const Real dt = Real(1) / 120;
    Real peak = 0, quarter = 0;
    for (int i = 0; i < 120; ++i) {
        rack_drive::drive(w, i * dt);
        w.step(dt);
        CHECK(!w.report().stalled && w.report().unfinished == 0, "rack did not finish frame %d", i+1);
        const Real t = (i+1)*dt;
        const Real expected = rack_drive::radius * kPi * std::min(t, 1-t);
        const Real measured = w.body(rack_drive::rack).position.x;
        peak = std::max(peak, std::abs(measured - expected));
        if (i == 59) quarter = measured;
    }
    std::printf("rack,fast=%d,quarter_m=%.12g,return_m=%.12g,max_error_m=%.12g\n",
                int(fast), quarter, w.body(rack_drive::rack).position.x, peak);
    CHECK_NEAR(peak, 0, 5e-4);
}

// Без сил у сферы постоянны v и ω. Независимо измеряем производные длины по трём позам произвольных анкеров.
void rodAnchorDerivatives() {
    World w;
    const auto a = w.addBody(Body::sphere(.2,1000,{1,2,3}));
    const auto b = w.addBody(Body::sphere(.3,800, {2,1,4}));
    const Vector3 va{.2,-.1,.3}, vb{-.4,.2,.1}, wa{.4,.7,-.3}, wb{.8,-.3,.2};
    const Vector3 armA{.1,.05,-.12}, armB{-.05,.02,.12};
    const Body& left = w.body(a); const Body& right = w.body(b);
    w.body(a).linearMomentum = (1/left.mass.inverseMass)*va;
    w.body(b).linearMomentum = (1/right.mass.inverseMass)*vb;
    w.body(a).angularMomentum = (1/left.mass.inverseInertia.x)*wa;
    w.body(b).angularMomentum = (1/right.mass.inverseInertia.x)*wb;
    const auto& rod = addDistance(w, a, left.position+armA, b, right.position+armB);
    const auto row = rod.bonds(w.bodies(), w.bodies()).front();
    const auto distanceAt = [&](Real t) {
        const Vector3 pa = left.position + t*va + Quaternion::fromRotationVector(t*wa).rotate(armA);
        const Vector3 pb = right.position + t*vb + Quaternion::fromRotationVector(t*wb).rotate(armB);
        return (pb-pa).length();
    };
    constexpr Real dt=1e-4;
    const Real derivative = row.linear.dot(vb-va) + row.angularB.dot(wb) - row.angularA.dot(wa);
    CHECK_NEAR(derivative, (distanceAt(dt)-distanceAt(-dt))/(2*dt), 1e-8);
    CHECK_NEAR(row.drift, (distanceAt(dt)-2*distanceAt(0)+distanceAt(-dt))/(dt*dt), 1e-6);
    w.body(b).position = left.position + armA - armB;
    const auto collapsed = rod.bonds(w.bodies(), w.bodies()).front();
    CHECK(std::isfinite(collapsed.drift) && std::isfinite(collapsed.error), "finite row for collapsed anchors");
    CHECK_NEAR(collapsed.error, -rod.length(), 1e-12);
}

template<class Create> bool rejected(Create create) {
    try { create(); }
    catch (const std::invalid_argument&) { return true; }
    return false;
}

void linkageEdges() {
    World w = rack_drive::build();
    auto& rack = w.body(rack_drive::rack);
    auto& pinion = w.body(rack_drive::pinion);
    const auto phase = [&] { return w.joints()[2].bonds(w.bodies(), w.bodies()).front().error; };
    rack.position.x = rack_drive::radius * 12.25 * 2 * kPi + .001;
    pinion.orientation = Quaternion::fromRotationVector((kPi/2) * rack_drive::shaft);
    CHECK_NEAR(phase(), -.001, 1e-10);
    pinion.orientation = {-pinion.orientation.w, -pinion.orientation.x, -pinion.orientation.y, -pinion.orientation.z};
    CHECK_NEAR(phase(), -.001, 1e-10);
    const auto a = rack_drive::rack, b = rack_drive::pinion;
    CHECK(rejected([&] { addDistance(w, a, {}, b, {}); }), "coincident rod anchors");
    CHECK(rejected([&] { addDistance(w, a, {}, a, {1,0,0}); }), "rod attached twice to one body");
    CHECK(rejected([&] { addRackAndPinion(w, a, b, {}, {1,0,0}, {1,0,0}, .02); }), "parallel rack and shaft");
    CHECK(rejected([&] { addRackAndPinion(w, a, b, {}, {0,1,0}, {1,0,0}, -1); }), "negative pitch radius");
    CHECK(rejected([&] { addRackAndPinion(w, a, b, {}, {}, {1,0,0}, .02); }), "empty shaft axis");
    CHECK(rejected([&] { addRackAndPinion(w, a, b, {}, {0,1,0}, {1,0,0}, std::numeric_limits<Real>::quiet_NaN()); }), "non-finite radius");
    CHECK(w.joints().size() == 3, "invalid linkage inserted");
}
}
void linkageTests() {
    rodAnchorDerivatives();
    linkageEdges();
    for (bool fast : {false, true}) {
        rotatingRod(fast);
        reversingRack(fast);
    }
}
