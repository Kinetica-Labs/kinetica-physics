#pragma once
// Строки решателя контактов (ContactSolver.h): точки касания, трение пар и связи сочленений — и как
// импульс вдоль строки меняет скорости тел. Общее состояние механизмов решателя: трения пары
// (PairFriction), сборки системы (ContactAssembly), решения вместе (CoupledSolve).

#include <collision/ContactPair.h>
#include <dynamics/ContactForces.h>
#include <dynamics/event_driven/ContactSystem.h>
#include <joints/JointSet.h>

#include <array>
#include <limits>
#include <vector>

namespace kp {

struct ContactRows {
    struct Row {
        ContactId key;
        Vector3 point, normal;
        Real separation, restitution;
        Real normalMass;   // 1 / (n · K n): сколько импульса даёт единицу скорости
        Real startSpeed;   // нормальная скорость при создании решателя
        Real drift;        // ускорение сближения точек двух тел без всяких сил, м/с²: см. geometricAcceleration
        Real roundA, roundB; // радиусы качения тел в месте контакта (ContactManifold)
        bool faceOnB;      // грань контакта принадлежит телу b
        bool touching;     // точка в полосе касания
    };
    using Slip = std::array<Real, 3>;                    // вдоль двух касательных и вокруг нормали
    using Response = std::array<std::array<Real, 3>, 3>; // матрица отклика трения пары
    // Строки одной пары тел и её трение. Трение пары приложено в одной точке — центре её точек
    // касания: сила в касательной плоскости и момент вокруг нормали, который мешает одному телу
    // вращаться по другому. Так же устроено трение в ReactPhysics3D
    // (ContactSolverSystem.cpp:445, «twist friction constraint at the center of the contact
    // manifold»). Неизвестных три — столько же, сколько у пары степеней свободы в плоскости
    // касания. Трение в каждой точке отдельно дало бы восемь неизвестных на те же три степени:
    // решение неоднозначно, и обход точек по очереди не сходится.
    struct Pair {
        size_t first, last; // строки: [first, last)
        int a, b;
        Vector3 normal, tangent1, tangent2, centre;
        Real friction;
        Real roundA, roundB; // радиусы качения тел в месте контакта (ContactManifold)
        Real reach;         // среднее расстояние точек от центра, м: переводит вращение вокруг нормали в скорость
        Response inverse;   // обратная матрица отклика: какой импульс гасит единичное скольжение
        Slip response;      // диагональ матрицы отклика: скольжение от единичного импульса
        Slip startSlip;     // скольжение при создании решателя, до действия сил
        Slip drift;         // ускорение скольжения без всяких сил: то же, что Row::drift, для трения
        Slip impulse;       // накопленный импульс трения: сила, сила, момент
    };
    using Line = ContactSystem::Line;

    // Медленнее пара считается не скользящей, м/с: для неё сначала пробуется трение покоя. Поднять
    // порог до уровня округления больших задач (10⁻⁷…10⁻⁶ м/с) нельзя: уже при 10⁻⁷ покоящаяся
    // пирамида из 36 кубов начинает расползаться. По той же причине нельзя и не замечать остановки
    // медленного скольжения: пара, ставшая скользящей, возвращается к трению покоя только событием
    // остановки.
    static constexpr Real kSliding = Real(1e-9);
    static constexpr Real kIdle = -std::numeric_limits<Real>::infinity(); // цель точки, не участвующей в фазе

    ContactRows(std::vector<Body>& bodies, const ContactForces* pressure, Vector3 up, Real restBounceSpeed, const JointSet* joints)
        : bodies(bodies), pressure(pressure), up(up), restBounceSpeed(restBounceSpeed), joints(joints) {}

    // Пересчитывает ускорения точек касания без сил по скоростям тел moving; возвращает наибольшее
    // из них, м/с².
    Real setDrift(const std::vector<Body>& moving);
    // Обратная эффективная масса вдоль направления d в точке point: 1 / (d · K d).
    static Real effectiveMass(const Body& a, const Body& b, Vector3 point, Vector3 d);
    Real relativeSpeed(const Row& row, Vector3 direction) const;
    void apply(const Row& row, Vector3 impulse);
    Line normalLine(const Row& row) const;
    Line frictionLine(const Pair& pair, int k) const;
    static Line bondLine(const Bond& bond) { return {bond.a, bond.b, bond.linear, bond.angularA, bond.angularB}; }
    void applyLine(const Line& line, Real impulse);
    Real speed(const Line& line) const;
    Real kineticEnergy() const;
    Real driveCap(size_t bond) const; // наибольший импульс привода в нынешней фазе

    std::vector<Body>& bodies;
    const ContactForces* pressure;
    Vector3 up;
    std::vector<Row> rows;
    std::vector<Pair> pairs;
    std::vector<Real> sum;      // накопленные нормальные импульсы строк
    std::vector<char> pressing; // активный набор: точки, где тела давят друг на друга
    std::vector<char> stuck;    // пары, которые не скользят: их трение решается вместе с нормалями
    Real restBounceSpeed;
    const JointSet* joints;
    std::vector<Bond> bonds;         // связи всех сочленений подряд
    std::vector<Real> bondImpulse;   // накопленные импульсы связей
    std::vector<Real> bondStart;     // скорость вдоль связи при создании решателя
    std::vector<Real> bondGoal;      // скорость вдоль связи, которой добивается нынешняя фаза
    std::vector<Real> bondBias;      // поправки к целям на отрезке опоры, см. ContactSolver::biasBonds
    std::vector<char> bondActive;    // связь участвует в нынешнем решении
    std::vector<char> bondPressing;  // активный набор пределов хода: сочленение давит на предел
    std::vector<signed char> saturated; // привод упёрся в предел силы: знак его импульса; 0 — не упёрся
    std::vector<int> bondFlips;      // сколько раз привод упирался и отпускался: перебор обязан кончиться
    bool holding = false; // решается опора на отрезке, а не мгновенный удар
    Real holdTime = 0;    // длина отрезка, на котором решается опора; при ударе ноль
    int iterations = 0;
};

} // namespace kp
