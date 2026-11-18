#pragma once
// Решение линейной системы с симметричной положительно определённой матрицей разложением
// Холецкого: A = L Lᵀ, затем две подстановки. Только сложение, умножение, деление и корень.

#include <kp/math/math_functions.h>

#include <algorithm>
#include <vector>

namespace kp {

// Симметричная матрица, хранимая по профилю: в строке i лежат элементы от столбца first[i] до
// диагонали. Левее профиля матрица нулевая, и множитель L разложения остаётся там нулевым,
// поэтому хранить и считать нужно только профиль. У задачи контакта, где неизвестные идут остров
// за островом и пара за парой вдоль стопки, профиль узкий: и память, и время растут как
// n · (ширина ленты)², а не как n² и n³.
struct ProfileMatrix {
    std::vector<int> first;    // первый хранимый столбец каждой строки
    std::vector<size_t> start; // где в values начинается строка
    std::vector<Real> values;

    int size() const { return int(first.size()); }
    // Добавляет следующую строку с профилем от столбца firstColumn; её элементы нулевые.
    void addRow(int firstColumn) {
        start.push_back(values.size());
        values.resize(values.size() + size_t(size() - firstColumn + 1), 0);
        first.push_back(firstColumn);
    }
    // Элемент строки i в столбце j, first[i] <= j <= i.
    Real& at(int i, int j) { return values[start[size_t(i)] + size_t(j - first[size_t(i)])]; }
};

// a — матрица (портится: в ней остаётся множитель L), b — правая часть, на выходе решение.
//
// Матрица может быть вырожденной: строка, которая линейно выражается через предыдущие, даёт
// нулевой ведущий элемент. Такая строка пропускается, её неизвестная полагается равной нулю.
// Для задачи контакта это четвёртая точка на грани, которую уже держат три: силу между четырьмя
// точками можно распределить по-разному при тех же скоростях, и здесь выбирается распределение
// без лишней точки. Ведущий элемент считается нулевым, если он меньше dependent от исходного
// диагонального.
inline void solveCholesky(ProfileMatrix& a, std::vector<Real>& b, Real dependent = Real(1e-14)) {
    const int n = a.size();
    thread_local std::vector<char> skipped; // рабочий массив потока: решение не выделяет память
    skipped.assign(size_t(n), 0);
    for (int i = 0; i < n; ++i) { // строка i множителя L
        const int from = a.first[size_t(i)];
        const Real original = a.at(i, i);
        for (int j = from; j < i; ++j) {
            if (skipped[size_t(j)]) {
                a.at(i, j) = 0;
                continue;
            }
            Real value = a.at(i, j);
            for (int k = std::max(from, a.first[size_t(j)]); k < j; ++k) value -= a.at(i, k) * a.at(j, k);
            a.at(i, j) = value / a.at(j, j);
        }
        Real diagonal = original;
        for (int k = from; k < i; ++k) diagonal -= a.at(i, k) * a.at(i, k);
        if (!(diagonal > dependent * original)) {
            // Пропущенная строка не должна влиять на остальные: её часть в L обнуляется.
            skipped[size_t(i)] = 1;
            for (int k = from; k < i; ++k) a.at(i, k) = 0;
            a.at(i, i) = 1;
            continue;
        }
        a.at(i, i) = sqrt(diagonal);
    }
    for (int i = 0; i < n; ++i) { // L y = b
        for (int k = a.first[size_t(i)]; k < i; ++k) b[size_t(i)] -= a.at(i, k) * b[size_t(k)];
        b[size_t(i)] = skipped[size_t(i)] ? Real(0) : b[size_t(i)] / a.at(i, i);
    }
    for (int i = n - 1; i >= 0; --i) { // Lᵀ x = y: найденное x[i] сразу вычитается из строк выше
        b[size_t(i)] = skipped[size_t(i)] ? Real(0) : b[size_t(i)] / a.at(i, i);
        for (int k = a.first[size_t(i)]; k < i; ++k) b[size_t(k)] -= a.at(i, k) * b[size_t(i)];
    }
}

} // namespace kp
