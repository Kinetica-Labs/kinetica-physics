#include <world/IslandSleep.h>

#include <collision/PairFinder.h>

#include <algorithm>

namespace kp {

void IslandSleep::track(size_t bodies) {
    asleep_.resize(bodies, 0), sleepingMass_.resize(bodies), quiet_.resize(bodies, 0), sleepIsland_.resize(bodies, -1);
}

int IslandSleep::count() const {
    int count = 0;
    for (char sleeps : asleep_) count += sleeps;
    return count;
}

void IslandSleep::wakeIsland(std::vector<Body>& bodies, int island) {
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (!asleep_[i] || sleepIsland_[i] != island) continue;
        bodies[i].mass = sleepingMass_[i];
        asleep_[i] = 0, quiet_[i] = 0, sleepIsland_[i] = -1;
    }
}

void IslandSleep::wake(std::vector<Body>& bodies, size_t body) {
    if (asleep(body)) wakeIsland(bodies, sleepIsland_[body]);
}

// Кого пора будить перед шагом. Спящее тело неподвижно, поэтому пара «бодрствующее — спящее»
// находится обычным поиском пар: если за шаг они могут коснуться, остров спящего просыпается.
void IslandSleep::wakeTouched(std::vector<Body>& bodies, const JointSet& joints, Vector3 gravity, Real dt, bool enabled) {
    bool any = false;
    for (char sleeps : asleep_) any = any || sleeps;
    if (!any) return;
    const bool everyone = !enabled || !(gravity.x == sleepGravity_.x && gravity.y == sleepGravity_.y && gravity.z == sleepGravity_.z);
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (!asleep_[i]) continue;
        const Body& body = bodies[i];
        const bool pushed = body.linearMomentum.dot(body.linearMomentum) + body.angularMomentum.dot(body.angularMomentum) > 0;
        if (everyone || pushed) wakeIsland(bodies, sleepIsland_[i]);
    }
    for (const Joint& joint : joints) {
        if (!joint.pushes()) continue;
        for (int body : {joint.a, joint.b})
            if (asleep_[size_t(body)]) wakeIsland(bodies, sleepIsland_[size_t(body)]);
    }
    // Спящего будит касание (то же, что объединяет тела в остров, settle) или сосед, который
    // движется быстрее порога сна и может долететь за кадр: летящий шар будит стопку до удара.
    // Стоящий сосед с зазором больше kContactDistance не будит: такие пары засыпали двумя островами
    // и будили друг друга по кругу — соседи мигали сном каждые полсекунды, хотя стояли.
    for (const ContactPair& pair : PairFinder(bodies, gravity).pairs(dt)) {
        const auto wakes = [&](int other) { return pair.inContact() || (!asleep_[size_t(other)] && quiet_[size_t(other)] == 0); };
        if (asleep_[size_t(pair.a)] && wakes(pair.b)) wakeIsland(bodies, sleepIsland_[size_t(pair.a)]);
        if (asleep_[size_t(pair.b)] && wakes(pair.a)) wakeIsland(bodies, sleepIsland_[size_t(pair.b)]);
    }
}

// Кому пора спать после шага: острова бодрствующих тел, в которых каждое тело медленнее порога
// дольше time. Остров с работающим приводом не засыпает.
// Время покоя каждого тела; true — какое-то покоится уже не меньше time.
bool IslandSleep::restingLongEnough(const std::vector<Body>& bodies, Real dt, Real speedLimit, Real time) {
    bool candidate = false;
    for (size_t i = 0; i < bodies.size(); ++i) {
        const Body& body = bodies[i];
        if (body.isStatic()) continue;
        const Real speed = body.linearVelocity().length() + body.angularVelocity().length() * (body.shape ? body.boundingRadius() : Real(0));
        quiet_[i] = speed < speedLimit ? quiet_[i] + dt : Real(0);
        candidate = candidate || quiet_[i] >= time;
    }
    return candidate;
}

// Усыпляет тела островов, которым спать можно: top — корень острова каждого тела, restless —
// пометка в корне острова, которому спать нельзя.
void IslandSleep::putToSleep(std::vector<Body>& bodies, const std::vector<int>& top, const std::vector<char>& restless) {
    std::vector<int> number(bodies.size(), -1);
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i].isStatic() || restless[size_t(top[i])]) continue;
        int& slept = number[size_t(top[i])];
        if (slept < 0) slept = sleptIslands_++;
        asleep_[i] = 1, sleepIsland_[i] = slept, sleepingMass_[i] = bodies[i].mass;
        bodies[i].mass = {};
        bodies[i].linearMomentum = bodies[i].angularMomentum = {};
    }
}

void IslandSleep::settle(std::vector<Body>& bodies, const JointSet& joints, Vector3 gravity, Real dt, Real speedLimit, Real time) {
    if (!restingLongEnough(bodies, dt, speedLimit, time)) return;
    std::vector<int> island(bodies.size());
    for (size_t i = 0; i < island.size(); ++i) island[i] = int(i);
    const auto root = [&](int body) {
        while (island[size_t(body)] != body) body = island[size_t(body)] = island[size_t(island[size_t(body)])];
        return body;
    };
    const auto join = [&](int x, int y) {
        if (bodies[size_t(x)].isStatic() || bodies[size_t(y)].isStatic()) return;
        const int a = root(x), b = root(y);
        island[size_t(std::max(a, b))] = std::min(a, b);
    };
    for (const ContactPair& pair : PairFinder(bodies, gravity).pairs(0))
        if (pair.inContact()) join(pair.a, pair.b);
    std::vector<char> restless(bodies.size(), 0); // остров, которому спать нельзя, помечен в корне
    for (const Joint& joint : joints) {
        join(joint.a, joint.b);
        if (joint.pushes()) restless[size_t(root(bodies[size_t(joint.a)].isStatic() ? joint.b : joint.a))] = 1;
    }
    std::vector<int> top(bodies.size());
    for (size_t i = 0; i < bodies.size(); ++i) {
        top[i] = root(int(i));
        if (bodies[i].isStatic()) continue;
        if (restless[i] && top[i] != int(i)) restless[size_t(top[i])] = 1;
        if (quiet_[i] < time) restless[size_t(top[i])] = 1;
    }
    putToSleep(bodies, top, restless);
    sleepGravity_ = gravity;
}

} // namespace kp
