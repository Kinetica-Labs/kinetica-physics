#pragma once
#include <world/World.h>
#include <array>
#include <chrono>
#include <cstdio>
class StageClock final : public kp::StepObserver {
    using Clock=std::chrono::steady_clock;
    Clock::time_point previous_;
    std::array<double,13> total_{};
public:
    void start() {previous_=Clock::now();}
    void store(size_t i) {
        const auto now=Clock::now();
        total_[i]+=std::chrono::duration<double,std::milli>(now-previous_).count();
        previous_=now;
    }
    void after(kp::StepStage s,const kp::World&) override {store(size_t(s));}
    ~StageClock() {
        auto* f=std::fopen("results/stage-cost.csv","w");
        if (!f) std::exit(4);
        const char* names[]={"Separation","Impact","Forces","Motion","Pairs","Contacts","WarmStart","Joints","Normals","Recheck","Friction","Memory","Remainder"};
        std::fprintf(f,"stage,total_ms\n");
        for(size_t i=0;i<total_.size();++i) std::fprintf(f,"%s,%.6f\n",names[i],total_[i]);
        std::fclose(f);
    }
};
