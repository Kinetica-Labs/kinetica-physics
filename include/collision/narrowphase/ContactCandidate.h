#pragma once
// Точка — кандидат в контакт, и порядок, в каком точки получает решатель.

#include <math/Vector3.h>

#include <cstdint>
#include <vector>

namespace kp {

// Вершина обрезаемого многоугольника.
struct ContactCandidate {
    Vector3 point;       // на грани, которую обрезают
    Real separation;  // расстояние до плоскости опорной грани
    std::uint32_t id;

    // Сначала опора — точки, которые уже касаются (зазор не больше touching или такой же, как у
    // самой глубокой), в порядке построения; за ними остальные по возрастанию зазора.
    static std::vector<ContactCandidate> ordered(const std::vector<ContactCandidate>& points, Real touching);
};

} // namespace kp
