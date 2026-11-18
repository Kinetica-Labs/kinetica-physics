#pragma once
// Малая задача дополнительности: найти λ ≥ 0, при котором w = A λ + q ≥ 0 и λᵢ wᵢ = 0.
// A — симметричная неотрицательно определённая матрица n × n по строкам.
//
// Главные повороты Мурти (Murty, «Note on a Bard-type scheme for solving the complementarity
// problem», Opsearch, 1974): набор ненулевых λ меняется по одному номеру — наименьшему из
// нарушивших условие; на наборе λ находится точно разложением Холецкого. Для положительно
// определённой A перебор конечен; перебор ограничен 2ⁿ шагами. Строка, чей ведущий элемент меньше
// kDependent от диагонального, считается зависимой и пропускается (её λ = 0): число
// обусловленности не больше 10⁴ — две почти совпавшие точки иначе дают импульсы ±10²⁰ (как Box2D,
// b2SolveContact: k_maxConditionNumber).
//
// Начальный набор — тёплый старт: на входе в lambda прошлый ответ; давят точки, у которых он больше
// нуля, и те, что сейчас сближаются (q < 0). Обычно ответ находится с первого разложения.

#include <kp/math/math_functions.h>

#include <algorithm>
#include <vector>

namespace kp {

constexpr Real kDependent = Real(1e-4);

// Плотное разложение Холецкого блока A по строкам rows, на месте; затем решение для rhs.
inline void solveDenseBlock(const std::vector<Real>& A, size_t n, const std::vector<size_t>& rows, std::vector<Real>& L, std::vector<Real>& rhs) {
    const size_t k = rows.size();
    L.assign(k * k, 0);
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

inline void solveComplementarity(const std::vector<Real>& A, const std::vector<Real>& q, std::vector<Real>& lambda) {
    const size_t n = q.size();
    thread_local std::vector<char> active; // рабочие массивы потока: решение не выделяет память
    thread_local std::vector<Real> L, rhs;
    thread_local std::vector<size_t> rows;
    active.resize(n);
    for (size_t i = 0; i < n; ++i) active[i] = lambda[i] > 0 || q[i] < 0;
    for (size_t round = 0; round < (size_t(1) << n); ++round) {
        rows.clear(), rhs.clear();
        for (size_t i = 0; i < n; ++i)
            if (active[i]) rows.push_back(i), rhs.push_back(-q[i]);
        solveDenseBlock(A, n, rows, L, rhs);
        std::fill(lambda.begin(), lambda.end(), Real(0));
        for (size_t r = 0; r < rows.size(); ++r) lambda[rows[r]] = rhs[r];
        size_t broken = n;
        for (size_t i = 0; i < n && broken == n; ++i) {
            Real w = q[i];
            for (size_t j = 0; j < n; ++j) w += A[i * n + j] * lambda[j];
            if (active[i] ? lambda[i] < 0 : w < 0) broken = i;
        }
        if (broken == n) return;
        active[broken] = !active[broken];
    }
    for (Real& value : lambda) value = std::max(value, Real(0));
}

} // namespace kp
