#pragma once
// Разложение Холецкого A = L Lᵀ симметричной положительно определённой матрицы, хранимое по профилю
// (как ProfileMatrix), которое не считается заново, когда у матрицы появляется или исчезает строка вместе
// со столбцом (Gill, Golub, Murray, Saunders, «Methods for modifying matrix factorizations», 1974).
//
// Новая последняя строка — одна подстановка L y = a по её профилю. Удаление строки p оставляет у строк
// ниже по лишнему элементу правее диагонали; они гасятся вращениями соседних столбцов (Гивенс), матрица
// L Lᵀ от вращений не меняется, а профиль строк не расширяется: левее профиля вращать нечего. Обе правки
// стоят порядка числа хранимых элементов L, разложение с нуля — те же добавления строк подряд. Только
// сложение, умножение, деление и корень.

#include <math/StepMemory.h>
#include <math/math_functions.h>

namespace kp {

class UpdatedCholesky {
public:
    int size() const { return int(first_.size()); }
    // Сколько чисел хранит L: мера цены одной правки и одного решения.
    size_t stored() const { return values_.size(); }
    void clear() { first_.clear(), start_.clear(), values_.clear(), skipped_.clear(); }
    // Новая последняя строка матрицы: row[j − first] — её элемент в столбце j, first <= j < size(),
    // левее first — нули; diagonal — элемент на диагонали. Строка, линейно выражающаяся через прежние
    // (ведущий элемент не больше dependent · diagonal), пропускается, как в solveCholesky: её
    // неизвестная полагается нулём.
    void append(int first, const StepArray<Real>& row, Real diagonal, Real dependent = Real(1e-14));
    // Убирает строку и столбец position; строки ниже сдвигаются на одну вверх.
    void remove(int position);
    // b — правая часть, на выходе решение A x = b.
    void solve(StepArray<Real>& b) const;

private:
    StepArray<int> first_;    // первый хранимый столбец каждой строки L
    StepArray<size_t> start_; // где в values_ начинается строка
    StepArray<Real> values_;  // строка i — её элементы от столбца first_[i] до диагонали
    StepArray<char> skipped_; // пропущенные строки
};

} // namespace kp
