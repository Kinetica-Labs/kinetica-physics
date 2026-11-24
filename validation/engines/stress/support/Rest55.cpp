// Воспроизведение сцены 7 окна с той же геометрией и настройками «считать как прибор».
#include <physics.h>
#include <math/WorkerPool.h>
#include "ContactOverlay.h"
#include <array>
#include <cstdio>
#include <cstdlib>

using namespace kp;
namespace {
World pyramid() {
    World w;
    w.addBody(Body::box({5,.5,5},0,{0,-.5,0}));
    for (int row=0; row<10; ++row)
        for (int k=0; k<10-row; ++k)
            w.addBody(Body::box({.2,.2,.2},1000,{Real(.4)*(Real(k)-Real(9-row)/2),Real(.2)+Real(.4)*row,0}));
    w.realtime=true; w.sleeping=false; w.sleepSpeed=.05; w.sleepTime=2; w.maxSegments=8;
    return w;
}
Real travel(const Transform& now, const Transform& past, Real radius) {
    const auto turn=now.orientation*past.orientation.conjugate();
    return (now.position-past.position).length()+2*radius*Vector3{turn.x,turn.y,turn.z}.length();
}
}

int main(int argc,char** argv) {
    WorkerPool::shared().resize(1);
    World world=pyramid();
    ContactOverlay overlay;
    std::array<Transform,56> last, anchor;
    for (size_t i=0; i<last.size(); ++i) last[i]=anchor[i]=world.bodies()[i].pose();
    const int limit=argc>1 ? std::atoi(argv[1]) : 2100;
    const bool pushTop=argc>2;
    std::puts("frame,speed_mm_s,tip_step_mm,tip_interval_mm,energy_J,kinetic_J,marker_count,tip_body,speed_body,corner_mm_s,corner_step_mm,corner_interval_mm,top_center_interval_mm");
    for(int f=0; f<=limit; ++f) {
        if(pushTop && f==1380) applyImpulse(world.body(BodyId(55)), {32,0,0}, world.body(BodyId(55)).position);
        if(f) world.step(Real(1)/60);
        Real speed=0,tip=0,drift=0,kinetic=0,energy=0;
        Real cornerSpeed=0,cornerStep=0,cornerTravel=0;
        size_t tipBody=0,speedBody=0;
        for(size_t i=1; i<last.size(); ++i){
            const auto& b=world.bodies()[i];
            const auto now=b.pose();
            if(f==1320) anchor[i]=now; // 22 с, до интервала из записи
            const Real delta=travel(now,last[i],b.boundingRadius());
            const Real velocity=b.linearVelocity().length()+b.boundingRadius()*b.angularVelocity().length();
            if(delta>tip){tip=delta;tipBody=i;}
            if(velocity>speed){speed=velocity;speedBody=i;}
            drift=std::max(drift,travel(now,anchor[i],b.boundingRadius()));
            for(Real x : {Real(-.2),Real(.2)}) for(Real y : {Real(-.2),Real(.2)}) for(Real z : {Real(-.2),Real(.2)}) {
                const Vector3 local{x,y,z}, arm=now.direction(local), point=now.position+arm;
                cornerSpeed=std::max(cornerSpeed,(b.linearVelocity()+b.angularVelocity().cross(arm)).length());
                cornerStep=std::max(cornerStep,(point-last[i].point(local)).length());
                cornerTravel=std::max(cornerTravel,(point-anchor[i].point(local)).length());
            }
            kinetic+=kineticEnergyOf(b);
            energy+=kineticEnergyOf(b)-world.gravity.dot(b.position)/b.mass.inverseMass;
            last[i]=now;
        }
        const int marks=int(overlay.sample(world).points.size());
        const Real topCenter=(world.bodies().back().position-anchor.back().position).length();
        std::printf("%d,%.12g,%.12g,%.12g,%.12g,%.12g,%d,%zu,%zu,%.12g,%.12g,%.12g,%.12g\n",
                    f,1000*speed,1000*tip,1000*drift,energy,kinetic,marks,tipBody,speedBody,
                    1000*cornerSpeed,1000*cornerStep,1000*cornerTravel,1000*topCenter);
    }
}
