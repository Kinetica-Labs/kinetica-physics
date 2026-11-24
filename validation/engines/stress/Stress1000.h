#pragma once
// STRESS_SCENE=pyramid1000 либо rain1000, STRESS_PASS=verify либо time.
// В каждом кадре 8 общих вызовов с интервалом 1/480 с. Измерения поз вынесены из таймера.
#include "StressReadings.h"
#include "CubeLayout.h"

namespace stress {
constexpr int frameCount = 120, substeps = 8;
constexpr double frameTime = 1.0/60;

inline SceneSpec build(bool rain) {
    SceneSpec s{rain ? "rain1000" : "pyramid1000", "1000 cubes", {{{20,.25,20},{0,-.25,0},0}}, {}, 2};
    forEachCube(rain, [&](double x, double y, double z, double vx, double vz) {
        s.boxes.push_back({{half,half,half},{x,y,z},64,{}, {vx,0,vz}});
    });
    return s;
}

inline std::uint64_t fingerprint(const std::vector<State>& state) {
    std::uint64_t hash = 14695981039346656037ull;
    absorb(hash, state);
    return hash;
}
inline void checkStart(const SceneSpec& s, const std::vector<State>& st) {
    require(st.size()==1001 && st.size()==s.boxes.size(), "expected 1000 dynamic cubes and a floor");
    for (size_t i=0; i<st.size(); ++i) {
        const auto& b=s.boxes[i]; const auto& p=st[i];
        require(len(p.at-b.at)<1e-7 && len(p.v-b.velocity)<1e-7 && len(p.w)<1e-7 &&
                std::abs(std::abs(p.turn.w)-1)<1e-7 && len({p.turn.x,p.turn.y,p.turn.z})<1e-7,
                "initial poses or velocities differ from the shared scene");
    }
}
inline void checkRulers() {
    State a{}, b{};
    a.turn = aboutZ(kScenePi/4);
    a.at.y = half*std::sqrt(2.)-.002;
    require(std::abs(bottom(a)+.002)<1e-12,"rotated floor-depth oracle");
    const BoxSpec cube{{half,half,half},{}};
    a = {};
    b.at.x = 2*half-.001;
    require(std::abs(penetration(cube,a,cube,b)-.001)<1e-12,"overlapping cube oracle");
    b.at.x += .002;
    require(penetration(cube,a,cube,b)==0,"separated cube oracle");
}
inline void frame(const Step& step) {
    for (int i=0; i<substeps; ++i) step(frameTime/substeps);
}

inline void timed(const Step& step, const Read& read) {
    std::array<double,frameCount> us{};
    for (auto& value : us) {
        const auto start = std::chrono::steady_clock::now();
        frame(step);
        value = std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
    }
    const auto finalHash = static_cast<unsigned long long>(fingerprint(read()));
    std::puts("frame,step_us,final_hash");
    for (int i=0; i<frameCount; ++i) std::printf("%d,%.3f,%016llx\n",i+1,us[size_t(i)],finalHash);
}

inline void verified(const SceneSpec& scene, bool rain, const Step& step, const Read& read) {
    std::FILE* positions = nullptr;
    if (const char* path = std::getenv("STRESS_POSES")) {
        positions = std::fopen(path, "w");
        require(positions != nullptr, "cannot open poses");
        std::fputs("frame,id,x_m,y_m,z_m,qw,qx,qy,qz\n",positions);
    }
    std::puts("frame,floor_mm,pair_mm,com_y_m,apex_drop_m,x_plus,x_minus,z_plus,z_minus,at_floor,below,asleep,energy_J,ballistic_mm,state_hash");
    for (int i=0; i<=frameCount; ++i) {
        if (i) frame(step);
        const auto st = read();
        const auto r = observe(scene, st, i*frameTime, rain && i<=12);
        std::printf("%d,%.9g,%.9g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%d,%d,%d,%.12g,%.9g,%016llx\n",
                    i,1000*r.floorDepth,1000*r.pairDepth,r.comHeight,r.apexDrop,r.edge[0],r.edge[1],r.edge[2],r.edge[3],
                    r.floorCount,r.below,r.asleep,r.energyJ,1000*r.freeFallError,
                    static_cast<unsigned long long>(fingerprint(st)));
        if (positions && (i==0 || i==30 || i==frameCount))
            for (size_t k=1; k<st.size(); ++k) {
                const auto p=st[k].at; const auto q=st[k].turn;
                std::fprintf(positions,"%d,%zu,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",i,k,p.x,p.y,p.z,q.w,q.x,q.y,q.z);
            }
    }
    if (positions) std::fclose(positions);
}

inline bool requested() { return std::getenv("STRESS_SCENE") != nullptr; }
inline void compareOrStress(const std::string& engine, const Simulate& simulate) {
    if (!requested()) return compare(engine, simulate);
    const std::string name = std::getenv("STRESS_SCENE");
    require(name=="pyramid1000" || name=="rain1000", "unknown stress scene");
    const char* pass = std::getenv("STRESS_PASS");
    require(pass && (!std::strcmp(pass,"time") || !std::strcmp(pass,"verify")), "unknown stress pass");
    const bool rain = name=="rain1000";
    const auto scene = build(rain);
    checkRulers();
    simulate(scene,[&](const Step& step, const Read& read) {
        checkStart(scene, read());
        if (!std::strcmp(pass,"time")) timed(step,read);
        else verified(scene,rain,step,read);
    });
}
} // namespace stress
