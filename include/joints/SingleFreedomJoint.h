#pragma once
// Сочленение с одной свободой — шарнир (поворот вокруг главной оси) или ползун (перемещение
// вдоль неё). По этой свободе работают привод, сухое трение, пружина с демпфером и пределы хода;
// всё это общее для обоих и собрано здесь. Наследник говорит только, какова его свобода.

#include <joints/Joint.h>

namespace kp {

// Расстояние до ближайшего предела хода и сторона, с которой он лежит.
struct LimitGap {
    Real gap, side; // side = +1 — нижний предел, −1 — верхний
};

class SingleFreedomJoint : public Joint {
public:
    // Привод по единственной свободе шарнира (поворот вокруг главной оси) или ползуна (движение
    // вдоль неё). У других сочленений привода нет.
    //   driveForce — момент, Н·м, или сила, Н: привод прикладывает её к телу b, а к телу a —
    //                противоположную, поэтому импульс и момент импульса пары он не меняет;
    //   holdSpeed  — привод держит скорость driveSpeed, рад/с или м/с, какой бы силы это ни
    //                потребовало. Скорость — тела b относительно тела a.
    // Привод положения собирается снаружи: скорость, пропорциональная отклонению coordinate() от
    // нужного значения.
    Real driveForce = 0;
    bool holdSpeed = false;
    Real driveSpeed = 0;
    // Наибольшая сила (момент) привода скорости. Упёршись в неё, привод давит с этой силой и
    // скорость уже не держит. За нулевое время конечная сила импульса не даёт, поэтому в ударе
    // привод с конечной силой не участвует; привод с бесконечной держит скорость и сквозь удар.
    Real driveStrength = std::numeric_limits<Real>::infinity();

    // Сопротивление движению по той же свободе — то, без чего сустав робота не описать:
    //   friction  — сухое трение, Н·м или Н: пока нагрузка меньше, сустав стоит; больше — движется,
    //               а трение давит против движения с этой силой. Устроено как привод скорости
    //               «ноль» с пределом силы friction;
    //   damping   — вязкое сопротивление c, Н·м·с/рад или Н·с/м: сила −c · скорость;
    //   stiffness — пружина k, Н·м/рад или Н/м, с положением покоя restPosition: сила
    //               −k · (координата − restPosition).
    // Пружина и демпфер считаются неявно — по положению и скорости в конце отрезка, — поэтому
    // устойчивы при любой жёсткости и любом шаге; цена — они гасят колебания чуть сильнее
    // настоящих (на долю порядка h · частота).
    Real friction = 0, damping = 0, stiffness = 0, restPosition = 0;

    // Пределы хода той же свободы: lower <= coordinate() <= upper. Предел устроен как контакт:
    // это односторонняя связь, которая только отталкивает от предела; момент, когда сочленение
    // до него дойдёт, — событие шага (EventSearch::earliestLimit), удар о предел неупругий.
    // Угол шарнира отсчитывается от −π до π, поэтому шарнир с пределом через ±π не проходит:
    // незаданный предел встаёт в ∓kHingeReach.
    Real lower = -std::numeric_limits<Real>::infinity(), upper = std::numeric_limits<Real>::infinity();

    // Координата свободы: угол шарнира, рад, или ход ползуна, м.
    virtual Real coordinate(const Body& bodyA, const Body& bodyB) const = 0;
    Real coordinate(const std::vector<Body>& bodies) const { return coordinate(bodies[size_t(a)], bodies[size_t(b)]); }
    // Свобода — поворот (шарнир), а не перемещение (ползун).
    virtual bool angular() const = 0;

    bool limited() const { return lower > -std::numeric_limits<Real>::infinity() || upper < std::numeric_limits<Real>::infinity(); }
    LimitGap limitGap(const Body& bodyA, const Body& bodyB) const;

    bool pushes() const override { return driveForce != 0 || (holdSpeed && driveSpeed != 0); }
    const SingleFreedomJoint* singleFreedom() const override { return this; }

protected:
    using Joint::Joint;
    // Привод, трение, пружина и предел вдоль линии свободы freedom.
    void addFreedomBonds(const Motion& motion, const Bond& freedom, std::vector<Bond>& bonds) const;
};

} // namespace kp
