#pragma once
// Сочленение по траектории: точка тела b идёт по кривой, закреплённой в теле a (в Jolt — PathConstraint).
// Свободно перемещение вдоль кривой; привод, трение, пружина и пределы по нему — SingleFreedomJoint,
// координата — длина дуги от начала кривой, м. У открытой кривой пределы хода сразу стоят на её концах.
//
// В ближайшей к точке тела b точке кривой берутся единичная касательная T и нормаль N (Path::normal).
// Две связи перемещения вдоль N и T × N держат точку на кривой: нарушение — отклонение от точки кривой,
// а в его второй производной, кроме слагаемых прямого ползуна, есть центростремительное: точка, идущая по
// кривой со скоростью v, обязана ускоряться к центру кривизны, d²C/dt² = … − (n · κN) v², κN — кривизна
// (Path::curvature). Кривая заново берётся в ближайшей точке при каждой сборке связей, поэтому её изгиб на
// шаге учтён.
//
// Поворот тела b — одно из трёх (PathRotation). «Вслед за кривой» запирает ориентацию тела относительно
// тройки кривой (T, N, T × N). Тройка поворачивается, пока точка идёт по кривой, на Ω радиан за метр
// (Path::turning), поэтому скорость такой связи — e · (ω_b − ω_a) − (e · Ω) T · (v_b − v_a): строка
// запертого поворота минус (e · Ω), умноженное на строку свободы вдоль кривой, а в её второй производной
// есть ещё −(e · dΩ/ds) v². Без этого слагаемого точный шаг дробил кадр на сотню отрезков (H106).

#include <joints/Path.h>
#include <joints/SingleFreedomJoint.h>

#include <memory>

namespace kp {

// Что сочленение делает с поворотом тела b (в Jolt — EPathRotationConstraintType).
enum class PathRotation {
    Free,       // тело вращается свободно: бусина, подвес
    Fixed,      // сохраняет ориентацию относительно тела a: вагонетка канатной дороги
    FollowPath, // поворачивается вместе с касательной и нормалью кривой: вагон на рельсах
};

class PathJoint final : public SingleFreedomJoint {
public:
    // anchor — точка тела b, которая идёт по кривой, мир; through — точки кривой в мире при нынешней позе
    // тела a (кривая закрепляется в нём); closed — кривая замкнута.
    PathJoint(int a, const Body& bodyA, int b, const Body& bodyB, Vector3 anchor, const std::vector<Vector3>& through, bool closed = false);
    std::unique_ptr<Joint> clone() const override { return std::make_unique<PathJoint>(*this); }

    PathRotation rotation = PathRotation::Free;
    // Кривая в осях тела a.
    const Path& path() const { return *path_; }

    using SingleFreedomJoint::coordinate;
    Real coordinate(const Body& bodyA, const Body& bodyB) const override;
    bool angular() const override { return false; }
    void applyDrive(std::vector<Body>& bodies, Real h) const override;

protected:
    void build(const Motion& motion, std::vector<Bond>& bonds) const override;

private:
    // Место точки на пути: параметр ближайшей точки кривой и координата свободы.
    struct Place {
        Real u, along;
    };
    Place placeOf(Vector3 local) const; // local — точка тела b в осях тела a
    Quaternion frameAt(Real u) const;   // тройка кривой в осях тела a
    void followPath(const Motion& motion, Real u, Real speed, const Bond& freedom, std::vector<Bond>& bonds) const;
    std::shared_ptr<const Path> path_;  // общая у копий сочленения
    Quaternion rider_;                  // тройка кривой в осях тела b: какой она была при создании
};

} // namespace kp
