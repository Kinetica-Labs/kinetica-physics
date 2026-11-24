#pragma once
// Решение системы линий в пространстве тел (тождество Вудбери): шесть неизвестных на тело вместо
// одной на линию.

#include <math/Span.h>
#include <math/StepMemory.h>
#include <dynamics/event_driven/ContactSystem.h>

#include <math/ProfileMatrix.h>

#include <vector>

namespace kp {

class BodySpaceSolve {
public:
    BodySpaceSolve(Span<Body> bodies, const ContactSystem& system) : bodies_(bodies), system_(system) {}
    // active — участвующие линии; cost — цена разложения в пространстве линий; x — правая часть,
    // на выходе решение. Возвращает false, если в пространстве линий дешевле: x не тронут.
    bool solve(const StepArray<size_t>& active, Real cost, StepArray<Real>& x) const;

private:
    // Тела участвующих линий: место тела в K (−1 — неподвижное или без линий), тела по порядку и
    // первое тело профиля каждой строки блоков.
    struct Layout {
        StepArray<int> place, listed, first;
    };
    bool lay(const StepArray<size_t>& active, Real cost, Layout& layout) const;
    ProfileMatrix massMatrix(const Layout& layout) const;
    static void rowOf(const ContactSystem::Line& line, int body, Real j[6]);
    void solveFor(const StepArray<size_t>& active, const Layout& layout, StepArray<Real>& x) const;
    StepArray<Real> residualOf(const StepArray<size_t>& active, const StepArray<Real>& b, const StepArray<Real>& x) const;

    Span<Body> bodies_;
    const ContactSystem& system_;
};

} // namespace kp
