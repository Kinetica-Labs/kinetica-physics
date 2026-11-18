#pragma once
// Точки контакта пары выпуклых форм. Способ зависит от вида пары: два многогранника, многогранник
// с круглой формой, две круглые формы.

#include <kp/collision/narrowphase/ContactManifold.h>
#include <kp/collision/narrowphase/Gjk.h>
#include <kp/collision/shapes/ConvexShape.h>

namespace kp {

class NarrowPhaseAlgorithm {
public:
    virtual ~NarrowPhaseAlgorithm() = default;
    // closest — ближайшие точки ядер форм; margin — точки дальше этого зазора в контакт не идут;
    // touching — полоса касания: точки в ней считаются опорой.
    // boundary — вершина, лежащая на границе грани с такой точностью, считается оставшейся внутри.
    virtual ContactManifold collide(const ConvexShape& a, const Transform& poseA, const ConvexShape& b, const Transform& poseB, const ClosestPoints& closest, Real margin,
                                    Real touching, Real boundary) const = 0;
    // Алгоритм для пары форм.
    static const NarrowPhaseAlgorithm& between(const ConvexShape& a, const ConvexShape& b);
};

} // namespace kp
