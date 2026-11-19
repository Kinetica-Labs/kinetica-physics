#pragma once
// Трение качения пары тел: момент в касательной плоскости контакта против относительного вращения,
// не больше arm · N; arm — плечо трения качения δ, м. Момент δ N не зависит от скорости, поэтому
// катящееся тело останавливается за конечное время: шар радиуса r на полу тормозит с ускорением
// 5 δ g / (7 r).

#include <dynamics/time_stepping/SolverBody.h>
#include <math/Matrix2x2.h>

namespace kp {

class RollingFriction {
public:
    // Отклик относительного вращения тел вокруг касательных t1 и t2.
    void prepare(const SolverBody& a, const SolverBody& b, Vector3 t1, Vector3 t2);
    void warmStart(SolverBody& a, SolverBody& b, Vector3 t1, Vector3 t2) const;
    // load — сумма нормальных импульсов точек контакта.
    void solve(SolverBody& a, SolverBody& b, Vector3 t1, Vector3 t2, Real arm, Real load);

private:
    Matrix2x2 mass_; // обратная матрица отклика вращения вокруг двух касательных
    Vector2 roll_;   // накопленный момент вокруг них
};

} // namespace kp
