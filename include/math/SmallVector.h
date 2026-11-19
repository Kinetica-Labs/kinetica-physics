#pragma once
// Массив со встроенным местом на N элементов: пока их не больше N, к куче он не обращается.
//
// Точки площадки контакта, вершины обрезаемой грани, вершины многогранника в осях соседа — массивы на
// несколько элементов, которые создаются десятки тысяч раз за кадр. У std::vector каждый такой
// массив — обращение к куче: у пирамиды из 36 кубов их было 61 тысяча на кадр (замер
// study-allocations). Здесь элементы лежат в самом объекте; если их стало больше N, массив
// переезжает в кучу и дальше ведёт себя как обычный. Элементы — только простые данные, которые можно
// копировать побайтно. Порядок элементов тот же, что у std::vector: на ответ движка замена не влияет.

#include <cstddef>
#include <cstring>
#include <new>
#include <type_traits>

namespace kp {

template <class T, size_t N> class SmallVector {
    static_assert(std::is_trivially_copyable<T>::value, "SmallVector хранит только побайтно копируемые данные");

public:
    SmallVector() {}
    SmallVector(const SmallVector& other) { *this = other; }
    SmallVector& operator=(const SmallVector& other) {
        if (this == &other) return *this;
        size_ = 0;
        reserve(other.size_);
        std::memcpy(static_cast<void*>(data_), other.data_, other.size_ * sizeof(T));
        size_ = other.size_;
        return *this;
    }
    ~SmallVector() {
        if (data_ != local()) ::operator delete(data_);
    }

    T* data() { return data_; }
    const T* data() const { return data_; }
    size_t size() const { return size_; }
    T& operator[](size_t i) { return data_[i]; }
    const T& operator[](size_t i) const { return data_[i]; }
    void push_back(const T& value) {
        const T copy = value; // value может лежать в этом же массиве и переехать при росте
        reserve(size_ + 1);
        new (data_ + size_++) T(copy);
    }
    void clear() { size_ = 0; }
    // Новые элементы — со значениями по умолчанию.
    void resize(size_t count) {
        reserve(count);
        for (size_t i = size_; i < count; ++i) new (data_ + i) T();
        size_ = count;
    }

private:
    T* local() { return reinterpret_cast<T*>(inline_); }
    void reserve(size_t count) {
        if (count <= capacity_) return;
        const size_t roomy = count > 2 * capacity_ ? count : 2 * capacity_;
        T* moved = static_cast<T*>(::operator new(roomy * sizeof(T)));
        std::memcpy(static_cast<void*>(moved), data_, size_ * sizeof(T));
        if (data_ != local()) ::operator delete(data_);
        data_ = moved, capacity_ = roomy;
    }

    alignas(T) unsigned char inline_[N * sizeof(T)];
    T* data_ = local();
    size_t size_ = 0, capacity_ = N;
};

// Обход и алгоритмы стандартной библиотеки: begin(v), end(v).
template <class T, size_t N> T* begin(SmallVector<T, N>& v) { return v.data(); }
template <class T, size_t N> T* end(SmallVector<T, N>& v) { return v.data() + v.size(); }
template <class T, size_t N> const T* begin(const SmallVector<T, N>& v) { return v.data(); }
template <class T, size_t N> const T* end(const SmallVector<T, N>& v) { return v.data() + v.size(); }

} // namespace kp
