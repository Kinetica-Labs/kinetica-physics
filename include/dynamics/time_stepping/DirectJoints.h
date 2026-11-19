#pragma once
// Запертые свободы всех сочленений, решённые вместе и точно: одна система K λ = −(J v + поправка),
// K = J M⁻¹ Jᵀ. У цепи и балки матрица ленточная, и разложение Холецкого по профилю
// (math/ProfileMatrix.h) стоит времени, пропорционального числу связей. Проходы по одной связи
// передают нагрузку вдоль цепи на звено за проход, поэтому мягкая связь под грузом расходилась:
// балка из шести брусков провисала на 58 мм.

#include <math/StepMemory.h>
#include <dynamics/time_stepping/SolverBody.h>
#include <joints/Bond.h>
#include <math/ProfileMatrix.h>

#include <vector>

namespace kp {

class DirectJoints {
public:
    void clear();
    // impulse — место накопленного импульса связи; оно переживает кадр.
    void add(const Bond& bond, Real* impulse);
    // Матрица отклика по нынешним скоростным свойствам тел; после неё связи не добавляются.
    void assemble(const StepArray<SolverBody>& states);
    void warmStart(StepArray<SolverBody>& states) const;
    // correction — доля нарушения положения, которую связь убирает за время h: скорость вдоль связи
    // после решения равна −correction · нарушение / h. Ноль — чистая скорость.
    void solve(StepArray<SolverBody>& states, Real correction, Real h);

private:
    void push(size_t row, StepArray<SolverBody>& states, Real impulse) const;

    std::vector<Bond> bonds_;
    StepArray<Real*> impulses_;
    ProfileMatrix response_;
};

} // namespace kp
