#pragma once
// Площадка касания.
//
// Два выпуклых тела касаются по пересечению своих опорных элементов — того, чем каждое упирается
// в общую касательную плоскость. Поэтому точки контакта строятся одним способом для всех фигур.
// У одного тела берётся плоский элемент: грань многогранника или торец цилиндра. У другого —
// контур, которым оно к этой плоскости обращено: грань, окружность торца, отрезок (ребро, ось
// капсулы, образующая цилиндра) или точка (центр шара). Точки контакта — вершины общей части
// контура и плоского элемента, трёх родов:
//   1) вершины контура над плоским элементом;
//   2) пересечения сторон контура с границей плоского элемента;
//   3) углы плоского элемента под контуром, если контур замкнут.
// Число точек меняется вместе с общей частью непрерывно: отдельного кода для «вершина на грани»,
// «ребро на торце» или «торец шире грани» нет.

#include <math/StepMemory.h>
#include <collision/narrowphase/ContactManifold.h>
#include <collision/shapes/Flat.h>
#include <collision/narrowphase/Gjk.h>
#include <collision/shapes/Outline.h>

#include <cstdint>
#include <optional>
#include <vector>

namespace kp {

// Площадка на плоском элементе тела reference, обращённом к телу other.
struct ContactPatch {
    // Точка ядра обращённого тела, вошедшая в площадку.
    struct Point {
        Vector3 core;
        std::uint32_t id;
    };
    Flat flat;
    Outline outline;
    StepArray<Point> points;
    Real gap = 0; // зазор между плоскостью элемента и самой низкой точкой другого тела

    static StepArray<Point> pointsOf(const Flat& flat, const Outline& outline);
    // Площадка на плоском элементе, которым reference обращено в сторону towards; у формы без
    // такого элемента её нет.
    static std::optional<ContactPatch> on(const ConvexShape& reference, const Transform& referencePose, const ConvexShape& other, const Transform& otherPose, Vector3 towards);
    // Точки площадки в виде контакта; onReferenceB — плоский элемент принадлежит телу B.
    ContactManifold manifold(bool onReferenceB, Real margin, Real touching) const;
    // Площадка пары, если тела касаются плоским элементом. Плоский элемент годится, когда его
    // плоскость отстоит от другого тела не дальше, чем истинное расстояние плюс полоса касания:
    // зазор плоскости g не больше расстояния d, и равен ему ровно тогда, когда касание — на самом
    // элементе. Строгое равенство выбирало бы между площадкой и одной точкой по шуму округления;
    // с полосой все точки, уже находящиеся в ней, попадают в контакт вместе. Из двух годных элементов
    // берётся тот, чья плоскость ближе к истине; при равенстве — элемент тела A.
    static ContactManifold between(const ConvexShape& a, const Transform& poseA, const ConvexShape& b, const Transform& poseB, const ClosestPoints& closest, Real margin, Real touching);
};

} // namespace kp
