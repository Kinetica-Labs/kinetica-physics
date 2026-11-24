// Link the existing grasp fixture against the accepted stock library on Windows.
#define main originalGripMain
#include "GripLimit.cpp"
#undef main
#include <array>
#include <chrono>
#include <cmath>
#include <string_view>

namespace {
using Clock=std::chrono::steady_clock;
using Stage=kp::StepStage;
constexpr size_t stageCount=size_t(Stage::Memory)+1;
struct ClockByStage final : kp::StepObserver {
    Clock::time_point last;
    std::array<Clock::duration,stageCount> sums{};
    void after(Stage stage,const kp::World&) override {
        const auto now=Clock::now();
        sums[size_t(stage)]+=now-last;
        last=now;
    }
};
double us(Clock::duration value) { return std::chrono::duration<double,std::micro>(value).count(); }
}
int main(int argc,char** argv) {
    if (argc!=2) return 2;
    const std::string_view phase=argv[1];
    const bool watch=phase=="observed", sleep=phase=="sleep";
    if (!watch && !sleep && phase!="awake") return 2;
    kp::WorkerPool::shared().resize(1);
    constexpr double dt=1.0/60, h=dt/8;
    constexpr grip::Load load{0.5,10,10};
    constexpr int repeats=24;
    std::array<Clock::duration,stageCount> stageTotal{};
    Clock::duration total{};
    int asleepSteps=0,maxAsleep=0,steps=0;
    double worstDrift=0,worstForce=0;
    for(int r=-3;r<repeats;++r) {
        Rig rig(1,load,true);
        rig.world().sleeping=sleep;
        rig.world().soft.coupledFriction=true;
        for(int f=0;f<15;++f) if(!rig.step(dt,10,0)) return 3;
        const double initial=rig.read().relativeY;
        ClockByStage stages;
        if(watch) rig.world().observer=&stages;
        for(int f=0;f<15;++f) {
            const auto before=rig.read();
            auto start=Clock::now();
            stages.last=start;
            for(int k=0;k<8;++k) {
                if(!rig.advance(h,10,-10)) return 4;
                if(r>=0) {
                    const int n=rig.world().report().asleep;
                    maxAsleep=std::max(maxAsleep,n);
                    asleepSteps+=int(n>0);
                    ++steps;
                }
            }
            const auto stop=Clock::now();
            const auto after=rig.read();
            if(!after.valid()) return 5;
            if(r>=0) {
                total+=stop-start;
                worstDrift=std::max(worstDrift,1000*std::abs(after.relativeY-initial));
                worstForce=std::max({worstForce,std::abs((after.leftPx-before.leftPx)/dt),std::abs((after.rightPx-before.rightPx)/dt)});
            }
        }
        if(r>=0) for(size_t i=0;i<stageCount;++i) stageTotal[i]+=stages.sums[i];
        rig.world().observer=nullptr;
    }
    constexpr int frames=15*repeats;
    std::printf("variant,frames,public_steps,steps_with_sleeper,max_asleep,us_frame,drift_mm,force_error_N,pairs_us,contacts_us,warm_us,joints_us,normals_us,recheck_us,motion_us,friction_us,memory_us\n");
    std::printf("%s,%d,%d,%d,%d,%.8f,%.12g,%.12g",argv[1],frames,steps,asleepSteps,maxAsleep,us(total)/frames,worstDrift,worstForce);
    for(auto s:{Stage::Pairs,Stage::Contacts,Stage::WarmStart,Stage::Joints,Stage::Normals,Stage::Recheck,Stage::Motion,Stage::Friction,Stage::Memory})
        std::printf(",%.8f",us(stageTotal[size_t(s)])/frames);
    std::puts("");
    return steps==8*frames && worstDrift<=.5 && worstForce<=.2 && (sleep || maxAsleep==0) ? 0 : 6;
}
