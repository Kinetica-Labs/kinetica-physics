#include <kp/body/PrincipalAxes.h>

namespace kp {
namespace {

using Matrix = Real[3][3];

// Собственные оси симметричной матрицы методом вращений Якоби: каждое вращение обнуляет один
// внедиагональный элемент; после нескольких кругов матрица диагональна, а произведение вращений —
// искомые оси (в столбцах axes). Число кругов фиксировано, чтобы результат не зависел от допуска;
// синус и косинус угла вращения выражаются через корень, без тригонометрии.
// Golub, Van Loan, «Matrix Computations», раздел 8.5.
void diagonalize(Matrix m, Matrix axes) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) axes[i][j] = i == j;
    for (int sweep = 0; sweep < 16; ++sweep) {
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                if (m[p][q] == 0) continue;
                const Real theta = (m[q][q] - m[p][p]) / (2 * m[p][q]);
                const Real t = (theta < 0 ? Real(-1) : Real(1)) / (abs(theta) + sqrt(theta * theta + 1));
                const Real c = 1 / sqrt(t * t + 1), s = t * c;
                for (int k = 0; k < 3; ++k) { // столбцы p и q
                    const Real kp = m[k][p], kq = m[k][q];
                    m[k][p] = c * kp - s * kq, m[k][q] = s * kp + c * kq;
                    const Real ap = axes[k][p], aq = axes[k][q];
                    axes[k][p] = c * ap - s * aq, axes[k][q] = s * ap + c * aq;
                }
                for (int k = 0; k < 3; ++k) { // строки p и q
                    const Real pk = m[p][k], qk = m[q][k];
                    m[p][k] = c * pk - s * qk, m[q][k] = s * pk + c * qk;
                }
            }
        }
    }
}

// Кватернион поворота, столбцы матрицы которого — оси x, y, z.
Quaternion orientationOf(Vector3 x, Vector3 y, Vector3 z) {
    // Берётся наибольшая из четырёх составляющих: деление на неё не теряет точности.
    const Real trace = x.x + y.y + z.z;
    if (trace > 0) {
        const Real w = sqrt(trace + 1) / 2;
        return Quaternion{w, (y.z - z.y) / (4 * w), (z.x - x.z) / (4 * w), (x.y - y.x) / (4 * w)}.unit();
    }
    if (x.x >= y.y && x.x >= z.z) {
        const Real qx = sqrt(1 + x.x - y.y - z.z) / 2;
        return Quaternion{(y.z - z.y) / (4 * qx), qx, (y.x + x.y) / (4 * qx), (z.x + x.z) / (4 * qx)}.unit();
    }
    if (y.y >= z.z) {
        const Real qy = sqrt(1 + y.y - x.x - z.z) / 2;
        return Quaternion{(z.x - x.z) / (4 * qy), (y.x + x.y) / (4 * qy), qy, (z.y + y.z) / (4 * qy)}.unit();
    }
    const Real qz = sqrt(1 + z.z - x.x - y.y) / 2;
    return Quaternion{(x.y - y.x) / (4 * qz), (z.x + x.z) / (4 * qz), (z.y + y.z) / (4 * qz), qz}.unit();
}

} // namespace

PrincipalAxes PrincipalAxes::of(Real inertia[3][3]) {
    Matrix axes;
    diagonalize(inertia, axes);
    const Vector3 x{axes[0][0], axes[1][0], axes[2][0]}, y{axes[0][1], axes[1][1], axes[2][1]};
    const Vector3 z = x.cross(y); // третья ось — так, чтобы тройка была правой
    return {orientationOf(x, y, z), {inertia[0][0], inertia[1][1], inertia[2][2]}};
}

} // namespace kp
