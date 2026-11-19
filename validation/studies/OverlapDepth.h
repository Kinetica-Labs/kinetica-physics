#pragma once
// Мерка перекрытия пары частей — общая для приборов сцен (StandardScenes.cpp, OverlapSpread.cpp).

#include <collision/ContactPair.h>

#include <cmath>

namespace kp {

// Глубина перекрытия пары частей. У призмы рельефа (CompoundBodyBuilder::ground) она меряется под
// наружной гранью и в пределах самой призмы — это точки площадки контакта. overlapDepth меряет от
// плоскости наружной грани без пределов: тело, лежащее на соседней призме за перегибом рельефа,
// оказывалось «вошедшим» на 19 мм, хотя в призму не входило (замер: холмы, тело 16 в покое).
inline double depthOf(const ContactPair& pair, const Body& a, const Body& b) {
    const ConvexHull *hullA = a.shape->hull(), *hullB = b.shape->hull();
    if (!hullA || !hullB || (hullA->outerFace < 0 && hullB->outerFace < 0)) return double(overlapDepth(a, b));
    double deepest = 0;
    for (int k = 0; k < pair.manifold.count; ++k) deepest = std::fmax(deepest, -double(pair.manifold.points[size_t(k)].separation));
    return deepest;
}

} // namespace kp
