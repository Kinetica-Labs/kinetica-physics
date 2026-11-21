#pragma once
// Сочленение по траектории: точка тела b идёт по кривой, закреплённой в теле a (в Jolt — PathConstraint).
// Свободно перемещение вдоль кривой; привод, трение, пружина и пределы по нему — SingleFreedomJoint,
// координата — длина дуги от начала кривой, м. У открытой кривой пределы хода сразу стоят на её концах.
//
// В ближайшей к точке тела b точке кривой берётся единичная касательная T. Две связи перемещения поперёк
// неё держат точку на кривой: нарушение — отклонение от точки кривой вдоль нормали n, а в его второй
// производной, кроме слагаемых прямого ползуна, есть центростремительное: точка, идущая по кривой со
// скоростью v, обязана ускоряться к центру кривизны, d²C/dt² = … − (n · κN) v², κN — кривизна (Path).
// Кривая заново берётся в ближайшей точке при каждой сборке связей, поэтому её изгиб на шаге учтён.

#include <joints/Path.h>
#include <joints/SingleFreedomJoint.h>

#include <memory>

namespace kp {

class PathJoint final : public SingleFreedomJoint {
public:
    // anchor — точка тела b, которая идёт по кривой, мир; through — точки кривой в мире при нынешней позе
    // тела a (кривая закрепляется в нём); closed — кривая замкнута.
    PathJoint(int a, const Body& bodyA, int b, const Body& bodyB, Vector3 anchor, const std::vector<Vector3>& through, bool closed = false);
    std::unique_ptr<Joint> clone() const override { return std::make_unique<PathJoint>(*this); }

    // Тело b сохраняет ориентацию относительно тела a (вагонетка); иначе вращается свободно (бусина).
    bool lockTurns = false;
    // Кривая в осях тела a.
    const Path& path() const { return *path_; }

    using SingleFreedomJoint::coordinate;
    Real coordinate(const Body& bodyA, const Body& bodyB) const override;
    bool angular() const override { return false; }
    void applyDrive(std::vector<Body>& bodies, Real h) const override;

protected:
    void build(const Motion& motion, std::vector<Bond>& bonds) const override;

private:
    Real placeOf(const Body& bodyA, const Body& bodyB) const; // параметр ближайшей точки кривой
    std::shared_ptr<const Path> path_; // общая у копий сочленения
};

} // namespace kp
