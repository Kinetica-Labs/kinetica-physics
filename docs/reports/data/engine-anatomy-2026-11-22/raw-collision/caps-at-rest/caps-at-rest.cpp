// Two cylinders, no loads and zero initial momenta. Exact motion is rest.
#include <world/World.h>
#include <collision/shapes/Cylinder.h>
#include <algorithm>
#include <array>
#include <cstdio>

using namespace kp;

Quaternion alignDirections(Vector3 from, Vector3 to) {
    const Vector3 axis = from.cross(to);
    return Quaternion{1 + from.dot(to), axis.x, axis.y, axis.z}.unit();
}

int main() {
    const Vector3 axis{.36,.48,.8}, radial{.8,0,.6}, up{0,1,0};
    const Vector3 rimNormal = .6 * radial + .8 * up;
    const Cylinder sharp(.23,.16), rounded(.23,.16,.025);
    const Vector3 rim = sharp.coreRadius * radial + sharp.coreHalfHeight * up + sharp.radius * rimNormal;
    const Vector3 cap = .3 * rounded.coreRadius * radial + (rounded.coreHalfHeight + rounded.radius) * up;
    const std::array<Transform,2> frames{{{{-4,2,3},Quaternion{1,-2,3,5}.unit()},
                                           {{2,7,-1},Quaternion{3,1,4,-2}.unit()}}};
    std::puts("frame,swap,gap_m,step_count,max_displacement_m,max_speed_m_s,max_spin_rad_s,max_kinetic_J,complete");
    for (int k=0; k<2; ++k) for (int swap : {0,1}) for (Real gap : {Real(0),Real(.00005)}) {
        const Transform& frame = frames[k];
        const Quaternion a = (frame.orientation * Quaternion::fromRotationVector(.37*axis) * alignDirections(rimNormal,axis)).unit();
        const Quaternion b = (frame.orientation * Quaternion::fromRotationVector(-.52*axis) * alignDirections(up,-axis)).unit();
        Body left = Body::cylinder(.23,.16,1000,frame.point({.125,-.25,.375}) - a.rotate(rim));
        Body right = Body::cylinder(.23,.16,1000,frame.point(Vector3{.125,-.25,.375} + gap*axis) - b.rotate(cap),.025);
        left.orientation=a;
        right.orientation=b;
        World world;
        world.gravity={};
        world.realtime=true;
        world.sleeping=false;
        world.soft.substeps = world.soft.contactUpdates = 1;
        const std::array<BodyId,2> ids{world.addBody(swap ? right : left), world.addBody(swap ? left : right)};
        Real shift=0, speed=0, spin=0, energy=0;
        int goodSteps=0;
        for (int i=0; i<480; ++i) {
            world.step(Real(1)/480);
            if (world.report().stalled || world.report().unfinished != 0) break;
            ++goodSteps;
            Real total=0;
            for (int j=0; j<2; ++j) {
                const Body& body=world.body(ids[j]);
                const Body& initial=((j==0) != (swap!=0)) ? left : right;
                shift=std::max(shift,(body.position-initial.position).length());
                speed=std::max(speed,body.linearVelocity().length());
                spin=std::max(spin,body.angularVelocity().length());
                total+=kineticEnergyOf(body);
            }
            energy=std::max(energy,total);
        }
        std::printf("%d,%d,%.9g,%d,%.12g,%.12g,%.12g,%.12g,%d\n",k,swap,double(gap),goodSteps,
                    double(shift),double(speed),double(spin),double(energy),int(goodSteps==480));
    }
}
