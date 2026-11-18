#pragma once
// Мир твёрдых тел. Шаг читается как оглавление: по строке на стадию.

#include <kp/joints/BallAndSocketJoint.h>
#include <kp/joints/CardanJoint.h>
#include <kp/joints/CylindricalJoint.h>
#include <kp/joints/FixedJoint.h>
#include <kp/joints/HingeJoint.h>
#include <kp/joints/JointSet.h>
#include <kp/joints/SliderJoint.h>
#include <kp/joints/TetherJoint.h>
#include <kp/collision/ContactPair.h>
#include <kp/collision/RayCast.h>
#include <kp/body/Body.h>
#include <kp/world/IslandSleep.h>
#include <kp/dynamics/event_driven/ContactSolver.h>
#include <kp/dynamics/event_driven/ExactStep.h>
#include <kp/dynamics/time_stepping/SoftStep.h>

#include <vector>

namespace kp {

enum class BodyId : int {};

// Прибор снаружи шага: таймер, учёт энергии, отрисовка. Физика о приборах не знает.
class World;
class StepObserver {
public:
    virtual ~StepObserver() = default;
    virtual void after(StepStage stage, const World& world) = 0;
};

class World {
public:
    Vector3 gravity{0, Real(-9.81), 0}; // м/с²
    StepObserver* observer = nullptr;
    // Проход «ударной волны» в решателе контактов (ContactSolver::solveRestingContacts). Гасит
    // остаток скольжения в стопках с большим отношением масс, но нарушает сохранение импульса;
    // выключен по умолчанию и нужен для сравнения.
    bool shockPropagation = false;
    // Наибольшее число отрезков в одном шаге; ноль — без предела. Шаг, которому отрезков не
    // хватило, кончается раньше dt (Report::unfinished): мир в нём отстаёт от часов, зато окно
    // с живой сценой не замирает на шаге с сотней событий.
    int maxSegments = 0;
    // Шаг с постоянной ценой кадра (dynamics/time_stepping/SoftStep.h): подшаги и проходы по контактам, как в
    // Box3D; настройки — soft. Приближённый; без этого флага шаг точный и служит эталоном.
    bool realtime = false;
    SoftStep soft;

    // Сон островов. Остров — тела, связанные касанием или сочленением. Если все тела острова
    // медленнее sleepSpeed дольше sleepTime, остров засыпает: его тела на время сна становятся
    // неподвижными (масса убирается, импульсы обнуляются) и не стоят шагу ничего. Остров
    // просыпается целиком, когда к нему может прикоснуться бодрствующее тело, когда его телу дали
    // импульс, когда включили привод его сочленения или изменили силу тяжести. Сон выключен по
    // умолчанию: он останавливает тело, которое двигалось, пусть и медленнее порога, — это
    // допуск, а не точность. Скорость тела — скорость его самой быстрой точки: |v| + |ω| R.
    // Неподвижное тело, которое передвинули из-под спящих, их не будит: разбудить нужно самому.
    bool sleeping = false;
    Real sleepSpeed = Real(1e-4); // м/с
    Real sleepTime = Real(0.5);   // с
    bool asleep(BodyId id) const { return sleep_.asleep(size_t(id)); }
    void wake(BodyId id) { sleep_.wake(bodies_, size_t(id)); }
    // Масса тела; у спящего — та, что вернётся к нему при пробуждении.
    const MassProperties& mass(BodyId id) const { return asleep(id) ? sleep_.mass(size_t(id)) : bodies_[size_t(id)].mass; }

    BodyId addBody(const Body& body) {
        bodies_.push_back(body);
        return BodyId(int(bodies_.size()) - 1);
    }
    // Сочленение двух тел (joints/): вид — класс-наследник Joint, прочие аргументы — как у его
    // конструктора после тел. Сочленённые тела друг с другом не сталкиваются: у них общая точка,
    // и их формы там обычно перекрыты. Возвращённая ссылка живёт, пока жив мир.
    template <class Kind, class... Rest> Kind& addJoint(BodyId a, BodyId b, Rest&&... rest) {
        return joints_.add(Kind(int(a), body(a), int(b), body(b), std::forward<Rest>(rest)...));
    }
    // Те же сочленения по общей точке и оси в мире, при нынешних позах тел.
    BallAndSocketJoint& addBallJoint(BodyId a, BodyId b, Vector3 anchor) { return addJoint<BallAndSocketJoint>(a, b, anchor); }
    HingeJoint& addHinge(BodyId a, BodyId b, Vector3 anchor, Vector3 axis) { return addJoint<HingeJoint>(a, b, anchor, axis); }
    SliderJoint& addSlider(BodyId a, BodyId b, Vector3 anchor, Vector3 axis) { return addJoint<SliderJoint>(a, b, anchor, axis); }
    CylindricalJoint& addCylindrical(BodyId a, BodyId b, Vector3 anchor, Vector3 axis) { return addJoint<CylindricalJoint>(a, b, anchor, axis); }
    FixedJoint& addWeld(BodyId a, BodyId b, Vector3 anchor) { return addJoint<FixedJoint>(a, b, anchor); }
    CardanJoint& addCardan(BodyId a, BodyId b, Vector3 anchor, Vector3 axis, Vector3 cross) { return addJoint<CardanJoint>(a, b, anchor, axis, cross); }
    // Упругая привязь точки pointA тела a к точке pointB тела b (точки — в мире).
    TetherJoint& addTether(BodyId a, Vector3 pointA, BodyId b, Vector3 pointB, Real stiffness, Real damping) {
        return joints_.add(TetherJoint(int(a), body(a), pointA, int(b), body(b), pointB, stiffness, damping));
    }
    const JointSet& joints() const { return joints_; }
    // Сочленение по порядку добавления, как его вид: world.joint<SliderJoint>(0).
    template <class Kind> Kind& joint(size_t index) { return dynamic_cast<Kind&>(joints_[index]); }
    // Первое тело на пути луча (collision/RayCast.h).
    RayHit castRay(const Ray& ray) const { return kp::castRay(bodies_, ray); }
    // Координата шарнира (рад) или ползуна (м).
    Real coordinate(const SingleFreedomJoint& joint) const { return joint.coordinate(bodies_); }

    Body& body(BodyId id) { return bodies_[size_t(id)]; }
    const Body& body(BodyId id) const { return bodies_[size_t(id)]; }
    const std::vector<Body>& bodies() const { return bodies_; }

    // Продвигает мир на dt: шагом с постоянной ценой кадра (realtime) или точным (ExactStep).
    void step(Real dt);

    // Что произошло за последний шаг.
    struct Report : ExactStep::Report {
        int asleep = 0; // сколько тел спит после шага
    };
    const Report& report() const { return report_; }

private:
    std::vector<Body> bodies_;
    JointSet joints_;
    ContactForces contactForces_; // силы в точках контакта с прошлого отрезка — для тёплого старта
    std::vector<std::vector<Real>> jointImpulses_; // импульсы связей сочленений с прошлого кадра (SoftStep)
    Report report_;
    // Пара, остановкой скольжения которой закончился прошлый отрезок: на следующем она не скользит.
    ContactSolver::SlidingStop stopped_;
    IslandSleep sleep_;
};

} // namespace kp
