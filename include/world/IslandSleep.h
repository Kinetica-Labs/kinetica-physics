#pragma once
// Сон островов. Остров — тела, связанные касанием или сочленением. Спящее тело на время сна
// неподвижно: масса убрана и хранится здесь, импульсы обнулены.

#include <body/Body.h>
#include <joints/JointSet.h>

#include <vector>

namespace kp {

class IslandSleep {
public:
    void track(size_t bodies);
    bool asleep(size_t body) const { return body < asleep_.size() && asleep_[body]; }
    // Масса, которая вернётся спящему телу при пробуждении.
    const MassProperties& mass(size_t body) const { return sleepingMass_[body]; }
    int count() const;
    void wake(std::vector<Body>& bodies, size_t body);
    void wakeTouched(std::vector<Body>& bodies, const JointSet& joints, Vector3 gravity, Real dt, bool enabled);
    void settle(std::vector<Body>& bodies, const JointSet& joints, Vector3 gravity, Real dt, Real speedLimit, Real time);

private:
    void wakeIsland(std::vector<Body>& bodies, int island);

    // Спит ли тело, его масса на время сна, сколько оно уже медленнее порога и остров, с
    // которым оно уснуло.
    std::vector<char> asleep_;
    std::vector<MassProperties> sleepingMass_;
    std::vector<Real> quiet_;
    std::vector<int> sleepIsland_;
    int sleptIslands_ = 0;
    Vector3 sleepGravity_;
};

} // namespace kp
