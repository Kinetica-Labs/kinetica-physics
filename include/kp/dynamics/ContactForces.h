#pragma once
// Силы в точках контакта, запомненные между отрезками шага: тёплый старт решателя.

#include <kp/math/Vector3.h>

#include <cstdint>
#include <map>

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
using ContactForces = std::map<ContactId, ContactForce>;

} // namespace kp
