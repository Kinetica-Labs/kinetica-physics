#pragma once
// Запертые свободы всех сочленений, решённые вместе и точно: одна система K λ = −(J v + поправка),
// K = J M⁻¹ Jᵀ. У цепи и балки матрица ленточная, и разложение Холецкого по профилю
// (math/ProfileMatrix.h) стоит времени, пропорционального числу связей. Проходы по одной связи
// передают нагрузку вдоль цепи на звено за проход, поэтому мягкая связь под грузом расходилась:
// балка из шести брусков провисала на 58 мм.

#include <kp/dynamics/time_stepping/SolverBody.h>
#include <kp/joints/Bond.h>
#include <kp/math/ProfileMatrix.h>

#include <vector>

namespace kp {

class DirectJoints {
public:
    void clear();
    // impulse — место накопленного импульса связи; оно переживает кадр.
    void add(const Bond& bond, Real* impulse);
    // Матрица отклика по нынешним скоростным свойствам тел; после неё связи не добавляются.
    void assemble(const std::vector<SolverBody>& states);
    void warmStart(std::vector<SolverBody>& states) const;
    // correction — доля нарушения положения, которую связь убирает за время h: скорость вдоль связи
    // после решения равна −correction · нарушение / h. Ноль — чистая скорость.
    void solve(std::vector<SolverBody>& states, Real correction, Real h);

private:
    void push(size_t row, std::vector<SolverBody>& states, Real impulse) const;

    std::vector<Bond> bonds_;
    std::vector<Real*> impulses_;
    ProfileMatrix response_;
};

} // namespace kp
