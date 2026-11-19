#pragma once
// Вещественное число движка. Одно определение на весь код: точность меняется здесь.
//
// Двойная точность — основная и единственная, которая проходит приёмку. Допуски алгоритмов
// выражены через kEpsilon, чтобы сборка с float оставалась возможной, но для неё их нужно
// проверять заново.
//
// Правило детерминизма: движок пользуется только сложением, вычитанием, умножением, делением
// и квадратным корнем — по IEEE 754 они округляются одинаково на любой машине. Синус, косинус,
// арктангенс и прочие функции стандартной библиотеки дают разные биты на разных платформах,
// поэтому в ядре не вызываются; когда они понадобятся, они будут написаны здесь же из тех
// же пяти операций. Правило проверяется сборкой: tests/check_no_libm.sh.

#include <cmath>
#include <limits>

namespace kp {

using Real = double;

constexpr Real kEpsilon = std::numeric_limits<Real>::epsilon();
constexpr Real kPi = Real(3.14159265358979323846264338327950288);

inline Real sqrt(Real x) { return std::sqrt(x); }
inline Real abs(Real x) { return std::fabs(x); }

// Синус и косинус без библиотеки: только сложение, умножение и сравнение.
//
// Библиотечные sin и cos на разных машинах и компиляторах округляют по-разному, а сложение и
// умножение по стандарту IEEE 754 дают одни и те же биты везде (слияние умножения со сложением
// запрещено флагом сборки). Поэтому здесь результат повторяется побитово на любой платформе.
//
// Угол приводится к отрезку [−π/4, π/4] вычитанием целого числа четвертей оборота; на этом отрезке
// синус и косинус считаются отрезками своих рядов Тейлора, которых хватает до последнего знака:
// первый отброшенный член синуса — (π/4)¹⁷ / 17! ≈ 5·10⁻¹⁷, косинуса — (π/4)¹⁸ / 18! ≈ 2·10⁻¹⁸.


struct SinCos {
    Real sin, cos;
};

// Угол в радианах, по модулю меньше миллиона оборотов.
inline SinCos sinCos(Real angle) {
    // Число четвертей оборота — ближайшее целое к angle · 2/π. π/2 вычитается двумя частями:
    // старшая часть записана с запасом нулевых младших битов, поэтому её произведение на число
    // четвертей точно, а остаток не теряет знаков (Cody, Waite, «Software Manual for the Elementary
    // Functions», 1980).
    constexpr Real kTwoOverPi = Real(0.63661977236758134308);
    constexpr Real kHalfPiHigh = Real(1.57079632673412561417), kHalfPiLow = Real(6.07710050650619224932e-11);
    const long long quarters = static_cast<long long>(angle * kTwoOverPi + (angle < 0 ? Real(-0.5) : Real(0.5)));
    const Real r = (angle - Real(quarters) * kHalfPiHigh) - Real(quarters) * kHalfPiLow;

    const Real z = r * r;
    const Real sine = r + r * z * (Real(-1) / 6 + z * (Real(1) / 120 + z * (Real(-1) / 5040 + z * (Real(1) / 362880 + z * (Real(-1) / 39916800 +
                      z * (Real(1) / 6227020800.0 + z * (Real(-1) / 1307674368000.0)))))));
    const Real cosine = 1 + z * (Real(-1) / 2 + z * (Real(1) / 24 + z * (Real(-1) / 720 + z * (Real(1) / 40320 + z * (Real(-1) / 3628800 +
                        z * (Real(1) / 479001600.0 + z * (Real(-1) / 87178291200.0 + z * (Real(1) / 20922789888000.0))))))));
    // Четверть оборота меняет синус и косинус местами со сменой знака.
    switch (quarters & 3) {
    case 0: return {sine, cosine};
    case 1: return {cosine, -sine};
    case 2: return {-sine, -cosine};
    default: return {-cosine, sine};
    }
}

// Угол точки (x, y) на плоскости, от −π до π, как atan2 стандартной библиотеки: обратное к sinCos,
// тоже без библиотеки. Грубая оценка арктангенса уточняется по своему же синусу и косинусу: для
// единичного вектора (c, s) sin(θ − a) = s · cos a − c · sin a, и это почти сама поправка θ − a.
// Ошибка за проход падает как её куб, делённый на шесть: с 0,005 рад до округления — за два
// прохода; третий — запас. Вне пространства kp вызывать как kp::atan2: глобальный ::atan2 из
// <cmath> для double с ним неразличим.
inline Real atan2(Real y, Real x) {
    const Real norm = sqrt(y * y + x * x);
    if (norm == 0) return 0;
    const Real s = y / norm, c = x / norm;
    const Real ax = c < 0 ? -c : c, ay = s < 0 ? -s : s;
    const Real t = ax > ay ? ay / ax : ax / ay;
    Real angle = t / (1 + Real(0.28) * t * t);
    if (ay > ax) angle = Real(1.57079632679489661923) - angle;
    if (c < 0) angle = kPi - angle;
    if (s < 0) angle = -angle;
    for (int pass = 0; pass < 3; ++pass) {
        const SinCos now = sinCos(angle);
        angle += s * now.cos - c * now.sin;
    }
    return angle;
}

} // namespace kp
