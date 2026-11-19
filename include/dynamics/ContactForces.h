#pragma once
// Силы в точках контакта, запомненные между отрезками шага: тёплый старт решателя.

#include <math/Vector3.h>

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

namespace kp {


// Какая это точка контакта: пара тел, номер точки в их многообразии и, у составных тел, части.
struct ContactId {
    int a, b;
    std::uint32_t id;
    int partA = 0, partB = 0;
    bool operator<(const ContactId& o) const {
        return a != o.a ? a < o.a : b != o.b ? b < o.b : partA != o.partA ? partA < o.partA : partB != o.partB ? partB < o.partB : id < o.id;
    }
};

// Сила в точке контакта: импульс за единицу времени. Хранится сила, а не импульс, потому что
// отрезки шага бывают разной длины.
struct ContactForce {
    Real normal = 0;
    Vector3 tangent;   // доля силы трения пары, мир
    Real twist = 0; // доля момента трения пары вокруг нормали
};
// Силы по ключу точки — одним массивом, упорядоченным по ключу. Между шагами массив не
// освобождается: очистка оставляет память за ним, и запись следующего шага к куче не обращается
// (у std::map каждая запись была обращением: четверть всех обращений шага).
class ContactForces {
public:
    using Entry = std::pair<ContactId, ContactForce>;
    void clear() { entries_.clear(); }
    // Сила точки key: новая запись встаёт на своё место по ключу, после записей с тем же ключом. Точки
    // приходят почти по порядку ключей, поэтому сдвигать приходится редко.
    ContactForce& operator[](const ContactId& key) {
        size_t at = entries_.size();
        entries_.push_back({key, {}});
        for (; at > 0 && key < entries_[at - 1].first; --at) entries_[at] = entries_[at - 1];
        entries_[at] = {key, {}};
        return entries_[at].second;
    }
    // Запись точки key — последняя из записанных с этим ключом — или end().
    const Entry* find(const ContactId& key) const {
        const auto after = std::upper_bound(entries_.begin(), entries_.end(), key, [](const ContactId& k, const Entry& e) { return k < e.first; });
        return after != entries_.begin() && !((after - 1)->first < key) ? &*(after - 1) : end();
    }
    const Entry* begin() const { return entries_.data(); }
    const Entry* end() const { return entries_.data() + entries_.size(); }

private:
    std::vector<Entry> entries_;
};

} // namespace kp
