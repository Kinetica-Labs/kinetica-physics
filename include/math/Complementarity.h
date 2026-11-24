#pragma once
// Малая задача дополнительности: найти λ ≥ 0, при котором w = A λ + q ≥ 0 и λᵢ wᵢ = 0.
// A — симметричная положительно определённая матрица n × n по строкам: вызывающий добавляет к
// диагонали проксимальную поправку (SoftContact::solveNormals), и ведущие элементы не опускаются
// ниже 23 % диагональных.
//
// Главные повороты Мурти (Murty, «Note on a Bard-type scheme for solving the complementarity
// problem», Opsearch, 1974): набор ненулевых λ меняется по одному номеру — наименьшему из
// нарушивших условие; на наборе λ находится точно разложением Холецкого. Для положительно
// определённой A перебор конечен; перебор ограничен 2ⁿ шагами. Для вырожденной A ответ не
// гарантирован: знак остатка w на зависимой строке — шум округления, строка то входит в набор, то
// выходит, и перебор исчерпывает свои шаги (19 % случайных входов, tests/PropertyTests.cpp).
// Строка, чей ведущий элемент меньше
// kDependent от диагонального, считается зависимой и пропускается (её λ = 0): число
// обусловленности не больше 10⁴ — две почти совпавшие точки иначе дают импульсы ±10²⁰ (как Box2D,
// b2SolveContact: k_maxConditionNumber).
//
// Начальный набор — тёплый старт: на входе в lambda прошлый ответ; давят точки, у которых он больше
// нуля, и те, что сейчас сближаются (q < 0). Обычно ответ находится с первого разложения.

#include <math/math_functions.h>

#include <algorithm>
#include <vector>

namespace kp {

constexpr Real kDependent = Real(1e-4);
constexpr size_t kSmallBlock = 8; // задаче до стольких строк рабочая память даётся со стека вызова

// Рабочая память задачи на n строк: разложение n × n, правая часть, номера давящих строк и признак «давит».
struct BlockMemory {
    Real* L;
    Real* rhs;
    size_t* rows;
    char* active;
};

// Плотное разложение Холецкого блока A (n × n по строкам) по k строкам rows, на месте; затем решение для rhs.
inline void solveDenseBlock(const Real* A, size_t n, const size_t* rows, size_t k, Real* L, Real* rhs) {
    std::fill(L, L + k * k, Real(0));
    for (size_t i = 0; i < k; ++i)
        for (size_t j = 0; j <= i; ++j) {
            Real value = A[rows[i] * n + rows[j]];
            for (size_t m = 0; m < j; ++m) value -= L[i * k + m] * L[j * k + m];
            if (i != j) L[i * k + j] = L[j * k + j] == 0 ? 0 : value / L[j * k + j];
            else L[i * k + i] = value > kDependent * A[rows[i] * n + rows[i]] ? sqrt(value) : 0; // 0 — зависимая строка
        }
    for (size_t i = 0; i < k; ++i) { // L y = b
        for (size_t m = 0; m < i; ++m) rhs[i] -= L[i * k + m] * rhs[m];
        rhs[i] = L[i * k + i] == 0 ? 0 : rhs[i] / L[i * k + i];
    }
    for (size_t i = k; i-- > 0;) { // Lᵀ x = y
        rhs[i] = L[i * k + i] == 0 ? 0 : rhs[i] / L[i * k + i];
        for (size_t m = 0; m < i; ++m) rhs[m] -= L[i * k + m] * rhs[i];
    }
}

inline void solveComplementarity(const Real* A, const Real* q, Real* lambda, size_t n, const BlockMemory& work) {
    for (size_t i = 0; i < n; ++i) work.active[i] = lambda[i] > 0 || q[i] < 0;
    for (size_t round = 0; round < (size_t(1) << n); ++round) {
        size_t pressing = 0;
        for (size_t i = 0; i < n; ++i)
            if (work.active[i]) work.rows[pressing] = i, work.rhs[pressing] = -q[i], ++pressing;
        solveDenseBlock(A, n, work.rows, pressing, work.L, work.rhs);
        std::fill(lambda, lambda + n, Real(0));
        for (size_t r = 0; r < pressing; ++r) lambda[work.rows[r]] = work.rhs[r];
        size_t broken = n;
        for (size_t i = 0; i < n && broken == n; ++i) {
            Real w = q[i];
            for (size_t j = 0; j < n; ++j) w += A[i * n + j] * lambda[j];
            if (work.active[i] ? lambda[i] < 0 : w < 0) broken = i;
        }
        if (broken == n) return;
        work.active[broken] = !work.active[broken];
    }
    for (size_t i = 0; i < n; ++i) lambda[i] = std::max(lambda[i], Real(0));
}

// Задача на n строк: A — n × n по строкам, q и lambda — по n чисел. Рабочая память — со стека вызова, если
// строк не больше kSmallBlock, иначе из массивов потока: решение не выделяет память. Площадку пары тел
// решатель зовёт десятки тысяч раз за кадр, и векторы потока, менявшие размер на каждом вызове, стоили
// заметной доли кадра (профиль VTune, H116).
inline void solveComplementarity(const Real* A, const Real* q, Real* lambda, size_t n) {
    if (n <= kSmallBlock) {
        Real L[kSmallBlock * kSmallBlock], rhs[kSmallBlock];
        size_t rows[kSmallBlock];
        char active[kSmallBlock];
        return solveComplementarity(A, q, lambda, n, {L, rhs, rows, active});
    }
    thread_local std::vector<Real> L, rhs;
    thread_local std::vector<size_t> rows;
    thread_local std::vector<char> active;
    L.resize(n * n), rhs.resize(n), rows.resize(n), active.resize(n);
    solveComplementarity(A, q, lambda, n, {L.data(), rhs.data(), rows.data(), active.data()});
}

inline void solveComplementarity(const std::vector<Real>& A, const std::vector<Real>& q, std::vector<Real>& lambda) {
    solveComplementarity(A.data(), q.data(), lambda.data(), q.size());
}

} // namespace kp
