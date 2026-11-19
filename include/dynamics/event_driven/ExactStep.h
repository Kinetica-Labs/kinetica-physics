#pragma once
// Точный шаг. Делится на отрезки в моменты событий: удар решается тогда, когда он произошёл, и
// остаток шага продолжается с новыми скоростями — время не теряется. Цена шага не ограничена.

#include <math/StepMemory.h>
#include <collision/ContactPair.h>
#include <joints/JointSet.h>
#include <dynamics/event_driven/ContactSolver.h>
#include <dynamics/event_driven/EventSearch.h>
#include <dynamics/event_driven/Rate.h>

#include <functional>
#include <utility>
#include <limits>
#include <vector>

namespace kp {

// Стадии отрезка шага в порядке выполнения. Наблюдатель получает управление после каждой.
enum class StepStage { Separation, Impact, Forces, Motion };

class ExactStep {
public:
    struct Report {
        int segments = 0;          // на сколько отрезков разделился шаг
        int solverIterations = 0;  // наибольшее число проходов решателя на отрезке
        // Чем обрывались отрезки: ударом (или пределом хода сочленения), остановкой скольжения,
        // пределом поворота тела в контакте.
        int impacts = 0, slidingStops = 0, turnLimits = 0;
        // Шаг остановлен раньше dt: два отрезка подряд не продвинули время и не изменили тела.
        // Продолжать нельзя — перекрытие началось бы сразу, — а повторять бесполезно: следующий
        // отрезок был бы тем же самым. Такое бывает, когда контакт не видит места, которым тела
        // сходятся; тела при этом остаются там, где были, а не входят друг в друга.
        bool stalled = false;
        Real unfinished = 0;       // сколько времени шага осталось не пройдено из-за maxSegments, с
    };

    // contactForces и stopped — память между шагами: силы в точках контакта с прошлого отрезка
    // (тёплый старт) и пара, остановкой скольжения которой он закончился.
    ExactStep(std::vector<Body>& bodies, const JointSet& joints, Vector3 gravity, ContactForces& contactForces, ContactSolver::SlidingStop& stopped, Report& report)
        : bodies_(bodies), joints_(joints), gravity_(gravity), contactForces_(contactForces), stopped_(stopped), report_(report), events_(bodies, joints) {}

    bool shockPropagation = false; // ContactSolver::solveRestingContacts
    int maxSegments = 0;           // наибольшее число отрезков; ноль — без предела
    std::function<void(StepStage)> observer;

    void run(Real dt);

private:
    // Силы на отрезке и момент, когда на нём остановится скольжение какой-нибудь точки.
    struct Forces {
        StepArray<Rate> rates;
        ContactSolver::SlidingStop slidingStop;
        StepArray<Rate> bonds; // доля сочленений в rates
        bool resolved = true;    // связи сочленений в конце отрезка выполнены
    };
    enum class Cut { None, Impact, SlidingStop, Turn }; // чем оборван отрезок
    Real advance(Real h);
    // Части advance: укорачивание отрезка по пределу поворота тела в контакте и до ближайшего события.
    void limitTurn(const std::vector<ContactPair>& pairs, Real& time, Forces& forces, Cut& cut);
    void cutAtEvents(const std::vector<ContactPair>& pairs, Real h, Real& time, Forces& forces, Cut& cut, ContactSolver::SlidingStop& stopped);
    void move(const std::vector<ContactPair>& pairs, const Forces& forces, Real time);
    std::vector<ContactPair> contacts(Real h) const;
    void resolveImpacts(const std::vector<ContactPair>& touching);
    Forces forcesWithin(const std::vector<ContactPair>& pairs, Real& time);
    Forces computeForces(const std::vector<ContactPair>& pairs, Real h);
    // Подбор сил связей сочленений по схеме SHAKE между проходами computeForces.
    struct BondShake {
        StepArray<Real> bias, errors, previousErrors;
        Real previousError = std::numeric_limits<Real>::infinity(), relaxation = 1;
    };
    static std::vector<Body> middleOf(const std::vector<Body>& before, const Forces& forces, Real h);
    bool forcesPass(const std::vector<ContactPair>& pairs, Real h, int pass, const std::vector<Body>& before, const StepArray<Real>& bias, Forces& forces);
    bool shakeBonds(BondShake& shake, const std::vector<Body>& before, Forces& forces, int pass, Real h) const;
    Vector3 up() const;
    Real restBounceSpeed() const;
    void notify(StepStage stage) {
        if (observer) observer(stage);
    }

    std::vector<Body>& bodies_;
    const JointSet& joints_;
    Vector3 gravity_;
    ContactForces& contactForces_;
    ContactSolver::SlidingStop& stopped_;
    Report& report_;
    EventSearch events_;
    // Пары, у которых в этом шаге уже была остановка скольжения (ContactSolver::slidingStop).
    StepArray<std::pair<int, int>> stoppedThisStep_;
};

} // namespace kp
