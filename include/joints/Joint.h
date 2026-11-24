#pragma once
// Сочленение: связь двух тел, запирающая часть из шести свобод их относительного движения —
// трёх перемещений и трёх поворотов. Все привычные сочленения — один и тот же набор запертых
// свобод с разными значениями:
//
//   сочленение            заперто перемещений   заперто поворотов   остаётся свобод
//   шаровое  (ball)       3                     0                   3 поворота
//   шарнир   (hinge)      3                     2                   поворот вокруг оси
//   карданное (cardan)    3                     1                   два поворота: вокруг оси y
//                                                                   тела a и вокруг оси z тела b;
//                                                                   кручение вала передаётся
//   ползун   (slider)     2                     3                   перемещение вдоль оси
//   цилиндрическое        2                     2                   поворот и перемещение по оси
//   жёсткое  (weld)       3                     3                   ни одной
//   по траектории (path)  2 поперёк кривой      0 или 3             перемещение вдоль кривой (PathJoint)
//
// У каждого тела своя точка сочленения и свои оси, заданные в осях тела; ось x — главная: ось
// шарнира, направление ползуна. Запертая свобода — линия связи (Bond): скорость вдоль неё обязана
// быть нулевой. Перемещение запирается в точке сочленения тела b: импульс связи приложен к обоим
// телам в одной и той же точке с противоположным знаком, поэтому суммарные импульс и момент
// импульса пары не меняются вовсе (proofs/bond_conserves_momenta.smt2). Линии связи решаются той же
// точной системой, что и контакты (ContactSolver): для неё связь — равенство, контакт —
// неравенство.
//
// Неподвижная опора — обычное тело с нулевой плотностью.
//
// Каждый вид сочленения — свой класс-наследник: он говорит, какие связи из общего набора ему
// нужны (build). Решатели про виды не знают и спрашивают только связи (bonds).

#include <body/Body.h>
#include <joints/Bond.h>

#include <memory>
#include <vector>

namespace kp {

class SingleFreedomJoint;

class Joint {
public:
    int a = 0, b = 0;               // номера тел
    Vector3 anchorA, anchorB;       // точка сочленения в осях каждого тела
    Quaternion frameA, frameB;      // оси сочленения в осях каждого тела; x — главная ось

    virtual ~Joint() = default;
    virtual std::unique_ptr<Joint> clone() const = 0;

    // Связи при нынешних позах тел bodies; уход связей считается по скоростям тел moving. У
    // сочленения двух неподвижных (или спящих) тел связей нет: двигать ими некого.
    std::vector<Bond> bonds(const std::vector<Body>& bodies, const std::vector<Body>& moving) const;
    // Перезаписывает result, сохраняя его запас для следующего сочленения.
    void writeBonds(const std::vector<Body>& bodies, const std::vector<Body>& moving, std::vector<Bond>& result) const;

    // Тела сочленения сталкиваются друг с другом. Обычно нет: они соединены вплотную.
    virtual bool collides() const { return false; }
    // Сочленение само двигает тела: привод или натянутая привязь. Такое не даёт острову уснуть.
    virtual bool pushes() const { return false; }
    // Постоянная сила привода за время h.
    virtual void applyDrive(std::vector<Body>&, Real) const {}
    // Сочленение с одной управляемой свободой — шарнир или ползун; у прочих nullptr.
    virtual const SingleFreedomJoint* singleFreedom() const { return nullptr; }

protected:
    Joint() = default;
    // anchor и axis — точка и главная ось сочленения в мире при нынешних позах тел.
    Joint(int a, const Body& bodyA, int b, const Body& bodyB, Vector3 anchor, Vector3 axis = {1, 0, 0});

    // Кратчайший поворот оси x в единичное направление to.
    static Quaternion turningXTo(Vector3 to);

    // Движение пары в точке сочленения: всё, из чего собираются связи.
    struct Motion {
        const Body *bodyA, *bodyB;              // тела в позах, для которых собираются связи
        Quaternion frameA, frameB;              // оси сочленения в мире
        Vector3 wa, wb, spinUpA, spinUpB;       // угловые скорости и ускорения тел без сил
        Vector3 pa, pb, armA, armB;             // точки сочленения и плечи до точки тела b
        Vector3 pointA, pointB, apart;          // скорости тел в этой точке и ускорение расхождения
    };
    virtual void build(const Motion& motion, std::vector<Bond>& bonds) const = 0;

    // Общий набор связей, из которого наследники берут нужные.
    Bond travelLine(const Motion& motion, int axis) const;  // перемещение вдоль оси сочленения
    Bond travelLine(const Motion& motion, Vector3 along) const; // то же вдоль единичного направления, которое несёт тело a
    Bond turnLine(const Motion& motion, int axis) const;    // поворот вокруг оси сочленения
    void lockTravel(const Motion& motion, int axis, std::vector<Bond>& bonds) const;
    void lockAllTurns(const Motion& motion, std::vector<Bond>& bonds) const;
    void lockTurnsAcrossAxis(const Motion& motion, std::vector<Bond>& bonds) const; // свободен только поворот вокруг главной оси
};

} // namespace kp
