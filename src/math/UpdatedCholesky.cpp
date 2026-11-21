#include <math/UpdatedCholesky.h>

#include <algorithm>
#include <vector>

namespace kp {

namespace {

// Σ a[k] b[k] четырьмя независимыми суммами: очередное сложение не ждёт предыдущего, и подстановка идёт
// вчетверо быстрее суммы по порядку. Порядок действий задан этим текстом и от компилятора не зависит.
inline Real dot(const Real* a, const Real* b, int count) {
    Real sum[4] = {0, 0, 0, 0};
    int k = 0;
    for (; k + 4 <= count; k += 4) sum[0] += a[k] * b[k], sum[1] += a[k + 1] * b[k + 1], sum[2] += a[k + 2] * b[k + 2], sum[3] += a[k + 3] * b[k + 3];
    for (; k < count; ++k) sum[0] += a[k] * b[k];
    return (sum[0] + sum[1]) + (sum[2] + sum[3]);
}

} // namespace

// Строка y множителя: L y = row, ведущий элемент — √(diagonal − y · y).
void UpdatedCholesky::append(int first, const StepArray<Real>& line, Real diagonal, Real dependent) {
    const int n = size();
    const size_t begin = values_.size();
    values_.resize(begin + size_t(n - first) + 1);
    Real* fresh = values_.data() + begin; // элемент столбца k — fresh[k − first]
    Real pivot = diagonal;
    for (int j = first; j < n; ++j) {
        const int from = first_[size_t(j)];
        const Real* above = values_.data() + start_[size_t(j)]; // элемент столбца k — above[k − from]
        const int low = std::max(first, from); // левее — нули у одной из строк
        const Real value = line[size_t(j - first)] - dot(above + (low - from), fresh + (low - first), j - low);
        fresh[j - first] = skipped_[size_t(j)] ? Real(0) : value / above[j - from];
        pivot -= fresh[j - first] * fresh[j - first];
    }
    const bool skip = !(pivot > dependent * diagonal);
    for (int j = first; skip && j < n; ++j) fresh[j - first] = 0; // пропущенная строка не влияет на остальные
    fresh[n - first] = skip ? Real(1) : sqrt(pivot);
    first_.push_back(first), start_.push_back(begin), skipped_.push_back(skip);
}

// Строка i > position становится строкой i − 1; её элементы — в прежних столбцах f … i, лишний — в
// столбце i. Вращение столбцов k и k + 1 с косинусом c и синусом s: (a, b) → (c a + s b, −s a + c b).
// Вращение k = i − 1 выбирается так, чтобы лишний элемент этой строки стал нулём, и затем применяется к
// строкам ниже. Диагональ остаётся положительной: r = √(a² + b²), где b — прежняя диагональ. У строки с
// f > position вращения начинаются со столбца f − 1 (левее — нули), и её профиль сдвигается на один.
void UpdatedCholesky::remove(int position) {
    const int n = size();
    thread_local std::vector<Real> cosine, sine; // рабочие массивы потока: правка не выделяет память
    cosine.assign(size_t(n), 1), sine.assign(size_t(n), 0);
    size_t write = start_[size_t(position)];
    for (int i = position + 1; i < n; ++i) {
        const int f = first_[size_t(i)], shifted = f <= position ? f : f - 1;
        const Real* from = values_.data() + start_[size_t(i)]; // элемент столбца k — from[k − f]
        Real* to = values_.data() + write;                     // to[k − shifted]; лежит раньше from: запись не обгоняет чтение
        for (int k = f; k < position; ++k) to[k - shifted] = from[k - f];
        Real carried = f <= position ? from[position - f] : Real(0);
        for (int k = std::max(position, f - 1); k < i - 1; ++k) {
            const Real next = from[k + 1 - f];
            to[k - shifted] = cosine[size_t(k)] * carried + sine[size_t(k)] * next;
            carried = cosine[size_t(k)] * next - sine[size_t(k)] * carried;
        }
        const Real diagonal = from[i - f], length = sqrt(carried * carried + diagonal * diagonal);
        cosine[size_t(i - 1)] = carried / length, sine[size_t(i - 1)] = diagonal / length;
        to[i - 1 - shifted] = length;
        first_[size_t(i - 1)] = shifted, start_[size_t(i - 1)] = write, skipped_[size_t(i - 1)] = skipped_[size_t(i)];
        write += size_t(i - shifted);
    }
    first_.pop_back(), start_.pop_back(), skipped_.pop_back(), values_.resize(write);
}

void UpdatedCholesky::solve(StepArray<Real>& b) const {
    const int n = size();
    for (int i = 0; i < n; ++i) { // L y = b
        const int from = first_[size_t(i)];
        const Real* at = values_.data() + start_[size_t(i)];
        const Real value = b[size_t(i)] - dot(at, b.data() + from, i - from);
        b[size_t(i)] = skipped_[size_t(i)] ? Real(0) : value / at[i - from];
    }
    for (int i = n - 1; i >= 0; --i) { // Lᵀ x = y: найденное x[i] сразу вычитается из строк выше
        const int from = first_[size_t(i)];
        const Real* at = values_.data() + start_[size_t(i)];
        b[size_t(i)] = skipped_[size_t(i)] ? Real(0) : b[size_t(i)] / at[i - from];
        for (int k = from; k < i; ++k) b[size_t(k)] -= at[k - from] * b[size_t(i)];
    }
}

} // namespace kp
