#include <collision/narrowphase/Gjk.h>
#include <cstdio>
#include <cmath>
#include <limits>

namespace kp {
using Query = ClosestPoints(const ConvexShape&, const Transform&, const ConvexShape&, const Transform&, Real);
Query originalProjection;
Query areaProjection;
bool originalShrank = false, areaShrank = false;

namespace {
struct Bounds { Real lower, upper; };
Bounds bounds(const ClosestPoints& answer, const ConvexShape& a, const Transform& pa,
              const ConvexShape& b, const Transform& pb) {
    if (answer.coresOverlap || answer.beyond) {
        const Real na = std::numeric_limits<Real>::quiet_NaN();
        return {na, na};
    }
    const Vector3 n = (1 / answer.normal.length()) * answer.normal;
    const Vector3 na = pa.localDirection(n), nb = pb.localDirection(-n);
    const Real lower = (pb.position-pa.position).dot(n) - na.dot(a.support(na))
                     - nb.dot(b.support(nb)) - a.radius - b.radius;
    return {lower, (answer.onB-answer.onA).length()};
}
} // namespace

ClosestPoints closestPoints(const ConvexShape& a, const Transform& pa, const ConvexShape& b,
                            const Transform& pb, Real farther) {
    originalShrank = areaShrank = false;
    const ClosestPoints original = originalProjection(a,pa,b,pb,farther);
    const ClosestPoints area = areaProjection(a,pa,b,pb,farther);
    const Bounds base = bounds(original,a,pa,b,pb), wide = bounds(area,a,pa,b,pb);
    const bool chooseArea = (wide.upper-wide.lower) < (base.upper-base.lower);
    std::fprintf(stderr,"bounds,%.17g,%.17g,%.17g,%.17g,%d,%d,%d\n",
                 double(base.lower),double(base.upper),double(wide.lower),double(wide.upper),
                 int(chooseArea),int(originalShrank),int(areaShrank));
    return chooseArea ? area : original;
}
} // namespace kp
