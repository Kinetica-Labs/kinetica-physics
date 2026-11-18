#pragma once
// Решение системы линий в пространстве тел (тождество Вудбери): шесть неизвестных на тело вместо
// одной на линию.

#include <kp/dynamics/event_driven/ContactSystem.h>

#include <kp/math/ProfileMatrix.h>

#include <vector>

namespace kp {

class BodySpaceSolve {
public:
    BodySpaceSolve(const std::vector<Body>& bodies, const ContactSystem& system) : bodies_(bodies), system_(system) {}
    // active — участвующие линии; cost — цена разложения в пространстве линий; x — правая часть,
    // на выходе решение. Возвращает false, если в пространстве линий дешевле: x не тронут.
    bool solve(const std::vector<size_t>& active, Real cost, std::vector<Real>& x) const;

private:
    // Тела участвующих линий: место тела в K (−1 — неподвижное или без линий), тела по порядку и
    // первое тело профиля каждой строки блоков.
    struct Layout {
        std::vector<int> place, listed, first;
    };
    bool lay(const std::vector<size_t>& active, Real cost, Layout& layout) const;
    ProfileMatrix massMatrix(const Layout& layout) const;
    static void rowOf(const ContactSystem::Line& line, int body, Real j[6]);
    void solveFor(const std::vector<size_t>& active, const Layout& layout, std::vector<Real>& x) const;
    std::vector<Real> residualOf(const std::vector<size_t>& active, const std::vector<Real>& b, const std::vector<Real>& x) const;

    const std::vector<Body>& bodies_;
    const ContactSystem& system_;
};

} // namespace kp
