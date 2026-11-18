#include <kp/joints/SingleFreedomJoint.h>

#include <algorithm>

namespace kp {

LimitGap SingleFreedomJoint::limitGap(const Body& bodyA, const Body& bodyB) const {
    Real low = lower, high = upper;
    if (angular()) low = std::max(low, -kHingeReach), high = std::min(high, kHingeReach);
    const Real at = coordinate(bodyA, bodyB);
    return high - at < at - low ? LimitGap{high - at, -1} : LimitGap{at - low, 1};
}

void SingleFreedomJoint::addFreedomBonds(const Motion& m, const Bond& freedom, std::vector<Bond>& bonds) const {
    // Привод: связь вдоль свободы с заданной скоростью.
    if (holdSpeed) {
        Bond drive = freedom;
        drive.driven = true, drive.speed = driveSpeed, drive.strength = driveStrength;
        bonds.push_back(drive);
    }
    // Сухое трение: скорость «ноль», пока хватает силы трения.
    if (friction > 0) {
        Bond rub = freedom;
        rub.driven = true, rub.strength = friction;
        bonds.push_back(rub);
    }
    if (stiffness > 0 || damping > 0) {
        Bond spring = freedom;
        spring.soft = true, spring.stiffness = stiffness, spring.damping = damping;
        spring.stretch = coordinate(*m.bodyA, *m.bodyB) - restPosition;
        bonds.push_back(spring);
    }
    // Предел хода: та же линия, повёрнутая от предела в разрешённую сторону.
    if (limited()) {
        const LimitGap limit = limitGap(*m.bodyA, *m.bodyB);
        Bond stop = freedom;
        stop.linear = limit.side * stop.linear, stop.angularA = limit.side * stop.angularA, stop.angularB = limit.side * stop.angularB;
        stop.drift = limit.side * stop.drift;
        stop.oneSided = true, stop.gap = limit.gap, stop.idle = limit.gap > kLimitBand;
        bonds.push_back(stop);
    }
}

} // namespace kp
