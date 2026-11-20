#pragma once
// Вид на массив: указатель и длина. Массивом не владеет и годится для любого std::vector — и
// обычного, и живущего до конца шага (StepArray): функции решателя берут тела через него.

#include <cstddef>
#include <vector>

namespace kp {

template <class T> class Span {
public:
    Span() = default;
    Span(const T* data, size_t size) : data_(data), size_(size) {}
    template <class Allocator> Span(const std::vector<T, Allocator>& all) : data_(all.data()), size_(all.size()) {}
    const T& operator[](size_t i) const { return data_[i]; }
    size_t size() const { return size_; }
    const T* begin() const { return data_; }
    const T* end() const { return data_ + size_; }

private:
    const T* data_ = nullptr;
    size_t size_ = 0;
};

} // namespace kp
