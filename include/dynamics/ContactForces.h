#pragma once
// Силы в точках контакта, запомненные между отрезками шага: тёплый старт решателя.

#include <math/Span.h>
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
    // Порядок: тела, части, номер точки — точки одной пары тел и частей стоят в массиве подряд.
    bool samePair(const ContactId& o) const { return a == o.a && b == o.b && partA == o.partA && partB == o.partB; }
    bool pairBefore(const ContactId& o) const { return a != o.a ? a < o.a : b != o.b ? b < o.b : partA != o.partA ? partA < o.partA : partB < o.partB; }
    bool operator<(const ContactId& o) const { return samePair(o) ? id < o.id : pairBefore(o); }
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
    // Записи всех точек пары тел и частей pair (номер точки в pair не читается), по порядку ключей.
    // Пары спрашивают по порядку, поэтому поиск начинается с hint — конца отрезка прошлой пары (в первый
    // раз begin()); двоичный поиск нужен, только когда следующая по порядку пара записей не оставила.
    Span<Entry> of(const ContactId& pair, const Entry* hint) const {
        const Entry* first = hint;
        if (first == end() || !first->first.samePair(pair))
            first = std::lower_bound(begin(), end(), pair, [](const Entry& e, const ContactId& k) { return e.first.pairBefore(k); });
        const Entry* last = first;
        while (last != end() && last->first.samePair(pair)) ++last;
        return {first, size_t(last - first)};
    }
    const Entry* begin() const { return entries_.data(); }
    const Entry* end() const { return entries_.data() + entries_.size(); }

private:
    std::vector<Entry> entries_;
};

} // namespace kp
