#pragma once
// Связь: одна запертая (или управляемая) свобода пары тел. Из связей состоит любое сочленение;
// решатели видят только их.

#include <math/Vector3.h>

#include <limits>

namespace kp {

// Запертая свобода сочленения. Скорость вдоль связи —
//   linear · (v_b − v_a) + angularB · ω_b − angularA · ω_a,
// и это производная нарушения связи по времени (tests/proofs/travel_bond_is_derivative.smt2,
// tests/proofs/hinge_bond_is_derivative.smt2).
struct Bond {
    int a, b;
    Vector3 linear, angularA, angularB;
    Real error; // насколько связь нарушена: метры у перемещения, радианы у поворота
    Real drift; // вторая производная нарушения без всяких сил: тела вращаются, и точка сочленения идёт по дуге
    // Связь привода: вместо «скорость вдоль связи нулевая» — «скорость вдоль связи равна speed».
    // Нарушения положения у неё нет. strength — наибольшая сила привода.
    bool driven = false;
    Real speed = 0;
    Real strength = std::numeric_limits<Real>::infinity();
    // Связь предела хода: односторонняя, её сила только отталкивает от предела. gap — сколько до
    // предела осталось (отрицательное — предел пройден); idle — до предела дальше полосы
    // kLimitBand, и связь не действует.
    bool oneSided = false;
    Real gap = 0;
    bool idle = false;
    // Связь пружины с демпфером: импульс за отрезок h равен −h (k (stretch + h v) + c v), где v —
    // скорость вдоль связи в конце отрезка, stretch — растяжение пружины в его начале.
    bool soft = false;
    Real stiffness = 0, damping = 0, stretch = 0;
};

// Сочленение у предела хода, когда до него осталось не больше этого: радианы и метры на равных.
// То же, что полоса касания у контакта.
constexpr Real kLimitBand = Real(1e-4);
// Крайние углы шарнира с пределами, рад: чуть меньше π.
constexpr Real kHingeReach = Real(3.13);

} // namespace kp
