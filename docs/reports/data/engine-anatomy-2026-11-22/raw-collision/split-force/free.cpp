#include <world/World.h>
#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace kp;
int main() {
    std::puts("hz,mass_kg,case,steps,max_position_m,max_velocity_m_s,complete");
    const Vector3 initial{1.,-1.5,.25}, v0{-.31,.4,.75};
    for (int hz : {60,120,240}) for (Real mass : {Real(1),Real(7)}) for (int k=0;k<4;++k) {
        World world;
        world.realtime=true;
        world.sleeping=false;
        world.soft.substeps=world.soft.contactUpdates=1;
        const Vector3 gravity=k==0 ? Vector3{} : Vector3{1,-2,3};
        const Vector3 applied=k==0 ? Vector3{0,-10,0} : k==1 ? Vector3{} : k==2 ? Vector3{-1,2,-3} : Vector3{4,0,-1};
        world.gravity=gravity;
        Body box=Body::box({.5,.5,.5},mass,initial);
        box.linearMomentum=mass*v0;
        const auto id=world.addBody(box);
        const Real h=Real(1)/(8*hz);
        const int steps=2*hz;
        Real error_x=0,error_v=0;
        bool complete=true;
        for (int i=1;i<=steps;++i) {
            world.body(id).force=mass*applied;
            world.step(h);
            complete=complete&&!world.report().stalled&&world.report().unfinished==0;
            const Real t=i*h;
            const Vector3 acc=gravity+applied;
            const Body& result=world.body(id);
            error_x=std::max(error_x,(result.position-(initial+t*v0+(.5*t*t)*acc)).length());
            error_v=std::max(error_v,(result.linearVelocity()-(v0+t*acc)).length());
        }
        std::printf("%d,%.0f,%d,%d,%.15g,%.15g,%d\n",hz,mass,k,steps,error_x,error_v,int(complete));
    }
}
