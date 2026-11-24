#define main previous_grip_main
#include "GripLimit.cpp"
#undef main
#include <array>
static std::array<unsigned long long,3> counts;
static bool record=false;
extern "C" void kp_record_lookup(int code) { if(record) ++counts[code]; }
int main(){
    kp::WorkerPool::shared().resize(1);
    std::puts("mode,case_name,phase,frame,public_steps,hits,misses,no_cache,relative_y_m,left_px_Ns,right_px_Ns");
    const grip::Load cases[]={{0.5,10,10},{0.5,10,12},{0.5,10,10,true}};
    const char* names[]={"threshold_10N","sliding_12N","open_10N"};
    for (bool coupled : {false,true}) for(int i=0;i<3;++i){
        const auto load=cases[i]; Rig rig(1,load,true);rig.world().soft.coupledFriction=coupled;
        for(int phase=0;phase<2;++phase) for(int frame=1;frame<=15;++frame){
            counts={};record=true;
            const bool ok=rig.step(1./60,load.squeeze,phase?load.down:0);
            record=false;
            const auto r=rig.read(); if (!ok || !r.valid()) return 1;
            std::printf("%s,%s,%s,%d,8,%llu,%llu,%llu,%.17g,%.17g,%.17g\n",coupled?"coupled":"stock",names[i],phase?"loaded":"preload",frame,counts[0],counts[1],counts[2],r.relativeY,r.leftPx,r.rightPx);
        }
    }
    return 0;
}
