// Отпускание наклонного ведра над двумя осевшими вёдрами. СИ, углы в выводе в градусах.
#include "StandardScenes.h"
#include <collision/PairFinder.h>
#include <math/WorkerPool.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string_view>

using namespace kp;
namespace {
constexpr Real h = Real(1) / 60;
constexpr BodyId upper{3};

bool touching(const World& w) {
    for (const auto& pair : PairFinder(w.bodies(), w.gravity).pairs(0))
        if ((pair.a == 3 || pair.b == 3) && pair.inContact()) return true;
    return false;
}

World place(const World& settled, Real angle, Real offset) {
    World w;
    w.realtime = true;
    w.sleeping = false;
    for (auto body : settled.bodies()) {
        body.linearMomentum = body.angularMomentum = {};
        body.linearDamping = body.angularDamping = 0;
        w.addBody(body);
    }
    Body& b = w.body(upper);
    b.orientation = Quaternion::fromRotationVector({0,0,angle * kPi / 180});
    b.position.x += offset;
    b.position.y = w.body(BodyId{2}).position.y + Real(.55);
    Real free = b.position.y;
    // Первый контакт при вертикальном опускании. Затем отпускание из покоя в 10 мкм от границы.
    while (!touching(w) && free - b.position.y < Real(.56)) b.position.y -= Real(.004);
    if (!touching(w)) throw std::runtime_error("bracket for first contact missing");
    Real contact = b.position.y;
    free = contact + Real(.004);
    for (int i = 0; i < 19; ++i) {
        b.position.y = (contact + free) / 2;
        if (touching(w)) contact = b.position.y;
        else free = b.position.y;
    }
    b.position.y = free + Real(.00001);
    return w;
}

double angleOf(const Body& b) {
    const auto up = b.orientation.rotate({0,1,0});
    return std::atan2(std::hypot(double(up.x),double(up.z)), double(up.y)) * 180 / double(kPi);
}

void trace(World& w, double offset) {
    for (int i=0; i<=86*60; ++i) {
        if (i>0) w.step(h);
        if (i<78*60) continue;
        double k=0, u=0, speed=0;
        for (const auto& b : w.bodies()) if (!b.isStatic()) {
            k += kineticEnergyOf(b);
            u -= w.gravity.dot(b.position) / b.mass.inverseMass;
            speed = std::max(speed,double(b.linearVelocity().length()+b.boundingRadius()*b.angularVelocity().length()));
        }
        std::printf("hold,%.4f,%d,%.17g,%.17g,%.17g,%.17g\n",offset,i,angleOf(w.body(upper)),k,u,speed);
    }
}
} // namespace

int main() {
    WorkerPool::shared().resize(1);
    for (const auto& scene: standardScenes()) if (std::string_view(scene.key) == "buckets") {
        auto base = scene.build();
        base.realtime = true;
        base.sleeping = false;
        for (int i=0; i<4*60; ++i) base.step(h);
        std::puts("mode,offset_m,frame,top_tilt_deg,kinetic_J,potential_J,point_speed_bound_m_s");
        double best = 1e9, selected = 0;
        for (double x : {-0.08,-0.05,-0.02,0.0,0.02,0.05,0.08}) {
            auto w = place(base,16,Real(x));
            for (int i=0; i<8*60; ++i) w.step(h);
            const double tilt = angleOf(w.body(upper));
            std::printf("search,%.4f,480,%.17g,,,\n",x,tilt);
            const double diff = std::abs(tilt-16);
            if (diff<best) { best=diff; selected=x; }
        }
        auto w = place(base,16,Real(selected));
        trace(w,selected);
        return 0;
    }
    return 2;
}
