#pragma once
// Набор сочленений мира. Владеет сочленениями разных видов и копируется вместе с ними: мир
// копируют целиком (снимок перед опытом, пробный шаг), и копия не должна делить сочленения с
// оригиналом.

#include <joints/Joint.h>

#include <memory>
#include <vector>

namespace kp {

class JointSet {
public:
    JointSet() = default;
    JointSet(const JointSet& other);
    JointSet(JointSet&&) = default;
    JointSet& operator=(JointSet other) {
        joints_.swap(other.joints_);
        return *this;
    }

    // Добавляет сочленение и возвращает его же: ссылка живёт, пока жив набор.
    template <class Kind> Kind& add(Kind joint) {
        joints_.push_back(std::make_unique<Kind>(std::move(joint)));
        return static_cast<Kind&>(*joints_.back());
    }

    size_t size() const { return joints_.size(); }
    bool empty() const { return joints_.empty(); }
    Joint& operator[](size_t index) { return *joints_[index]; }
    const Joint& operator[](size_t index) const { return *joints_[index]; }

    // Обход: for (const Joint& joint : set).
    class Iterator {
    public:
        explicit Iterator(const std::unique_ptr<Joint>* at) : at_(at) {}
        const Joint& operator*() const { return **at_; }
        Iterator& operator++() { return ++at_, *this; }
        bool operator!=(const Iterator& other) const { return at_ != other.at_; }

    private:
        const std::unique_ptr<Joint>* at_;
    };
    Iterator begin() const { return Iterator(joints_.data()); }
    Iterator end() const { return Iterator(joints_.data() + joints_.size()); }

    // Наибольшее нарушение связей: расхождение точек, перекос осей, проход за предел хода.
    Real largestError(const std::vector<Body>& bodies) const;

private:
    std::vector<std::unique_ptr<Joint>> joints_;
};

} // namespace kp
