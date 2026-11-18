#include <kp/dynamics/event_driven/ExactStep.h>

#include <kp/collision/PairFinder.h>
#include <kp/dynamics/Integrator.h>
#include <kp/dynamics/event_driven/OverlapSeparation.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace kp {
namespace {

// Сколько раз отрезок укорачивается до ближайшего события — удара или остановки скольжения.
constexpr int kMaxRefinements = 8;

// На сколько радиан тело, которое чего-то касается, может повернуться за один отрезок. Силы
// контакта на отрезке постоянны и найдены по геометрии его начала; для тела, успевающего
// провернуться, они теряют смысл. Стержень, после удара на 10 км/с вращающийся со скоростью
// 10⁵ рад/с, получал на отрезке в 1/60 с «угловую скорость» 3·10¹¹ рад/с.
constexpr Real kMaxContactTurn = 1;

// Поправка скорости от вращения тел, ниже которой второй проход расчёта сил не нужен, м/с.
constexpr Real kDriftResolved = Real(1e-10);

// Нарушение связи сочленения в конце отрезка, ниже которого силы считаются найденными: метры и
// радианы. И сколько раз силы уточняются ради этого.
constexpr Real kBondResolved = Real(1e-11);
constexpr int kMaxBondPasses = 12;

bool unchanged(const Body& x, const Body& y) {
    const auto same = [](Vector3 p, Vector3 q) { return p.x == q.x && p.y == q.y && p.z == q.z; };
    return same(x.position, y.position) && same(x.linearMomentum, y.linearMomentum) && same(x.angularMomentum, y.angularMomentum) &&
           x.orientation.w == y.orientation.w && x.orientation.x == y.orientation.x && x.orientation.y == y.orientation.y && x.orientation.z == y.orientation.z;
}

} // namespace

void ExactStep::run(Real dt) {
    int idle = 0; // сколько отрезков подряд не продвинули время и не изменили тела
    for (Real remaining = dt; remaining > 0 && !report_.stalled; ++report_.segments) {
        if (maxSegments > 0 && report_.segments == maxSegments) {
            report_.unfinished = remaining;
            break;
        }
        const std::vector<Body> before = bodies_;
        const Real passed = advance(remaining);
        remaining -= passed;
        idle = passed == 0 && std::equal(bodies_.begin(), bodies_.end(), before.begin(), unchanged) ? idle + 1 : 0;
        report_.stalled = idle == 2;
    }
    // Вторую половину толчка связей даёт удар в начале следующего отрезка (см. advanced); чтобы
    // и в конце шага скорости тел не шли вдоль связей, он решается здесь.
    if (!joints_.empty()) resolveImpacts(contacts(0));
}

std::vector<ContactPair> ExactStep::contacts(Real h) const { return PairFinder(bodies_, gravity_).pairs(h, joints_); }

// Один отрезок шага длиной не больше h; возвращает, сколько времени пройдено.
Real ExactStep::advance(Real h) {
    std::vector<ContactPair> touching = contacts(0);
    if (OverlapSeparation(bodies_, joints_, up()).separate(touching)) touching = contacts(0);
    notify(StepStage::Separation);
    resolveImpacts(touching);
    notify(StepStage::Impact);

    const std::vector<ContactPair> pairs = contacts(h);
    Real time = h;
    Forces forces = forcesWithin(pairs, time);
    // Отрезок укорачивается, пока тело в контакте поворачивается на нём больше предела. Силы,
    // найденные на длинный отрезок для быстро вращающегося тела, огромны, и поворот от них растёт
    // куда быстрее времени: деление длины на «во сколько раз превышен предел» бросало отрезок с
    // 1/60 с сразу на пикосекунды, хотя при нынешней угловой скорости ω телу хватило бы предел / ω.
    // Поэтому короче половины этого времени отрезок за один раз не делается.
    enum class Cut { None, Impact, SlidingStop, Turn } cut = Cut::None; // чем оборван отрезок
    for (Real turn; (turn = events_.contactTurn(pairs, forces.rates, time)) > kMaxContactTurn;) {
        const Real atPresentSpin = events_.contactTurn(pairs, std::vector<Rate>(bodies_.size()), time); // поворот без сил
        const Real unforced = atPresentSpin > 0 ? time * kMaxContactTurn / atPresentSpin : time;
        time = std::max(time * std::min(Real(0.5), kMaxContactTurn / turn), std::min(time, unforced) / 2);
        forces = forcesWithin(pairs, time);
        cut = Cut::Turn;
    }
    ContactSolver::SlidingStop stopped;
    // Событие — удар или остановка скольжения — обрывает отрезок раньше, чем кончается h. Силы
    // найдены так, чтобы точки контакта не сместились за всё h; приложенные лишь на часть этого
    // времени, они совершают работу и добавляют телам энергию. Поэтому отрезок укорачивается до
    // события, а силы пересчитываются на новую длину — пока внутри отрезка событий не останется.
    // Остановка скольжения, до которой на часах отрезка не дойти (h − t = h), считается уже
    // наступившей и событием не является. Удар — жёсткая граница: отрезок не идёт дальше него
    // никогда, даже если удар наступает сразу и отрезок выходит нулевым.
    bool verified = false; // при нынешних силах на отрезке нет событий
    for (int refinement = 0; refinement < kMaxRefinements && time > 0 && !verified; ++refinement) {
        const Real impact = std::min(events_.earliestImpact(pairs, forces.rates, time), events_.earliestLimit(forces.rates, time));
        const bool slides = forces.slidingStop.time <= impact && h - forces.slidingStop.time != h;
        const Real event = slides ? forces.slidingStop.time : impact;
        verified = event >= time;
        if (verified) break;
        stopped = slides ? forces.slidingStop : ContactSolver::SlidingStop{};
        cut = slides ? Cut::SlidingStop : Cut::Impact;
        time = event;
        if (time > 0) forces = forcesWithin(pairs, time);
    }
    // Силы, найденные последними, на удары ещё не проверены: укорачивания кончились раньше.
    if (!verified && time > 0) {
        const Real impact = std::min(events_.earliestImpact(pairs, forces.rates, time), events_.earliestLimit(forces.rates, time));
        if (impact < time) time = impact, cut = Cut::Impact;
    }
    report_.impacts += cut == Cut::Impact, report_.slidingStops += cut == Cut::SlidingStop, report_.turnLimits += cut == Cut::Turn;
    if (cut == Cut::SlidingStop) stoppedThisStep_.push_back({stopped.a, stopped.b});
    stopped_ = stopped;
    const std::vector<Rate>& rates = forces.rates;
    notify(StepStage::Forces);

    for (size_t i = 0; i < bodies_.size(); ++i)
        bodies_[i] = advanced(bodies_[i], rates[i].force, rates[i].torque, forces.bonds[i].force, forces.bonds[i].torque, time);
    notify(StepStage::Motion);
    return time;
}

// Удары: тела, которые уже касаются и сближаются, мгновенно меняют скорости. Внешние силы в этом
// не участвуют — за нулевое время они ничего не успевают.
void ExactStep::resolveImpacts(const std::vector<ContactPair>& touching) {
    // Два прохода: первый находит, на какие точки пришёлся удар, второй прикладывает трение в
    // центре этих точек. Если удара не было, второй проход не нужен.
    const std::vector<Body> before = bodies_;
    ContactSolver first(bodies_, touching, up(), restBounceSpeed(), &contactForces_, &joints_);
    first.solveImpacts();
    report_.solverIterations = std::max(report_.solverIterations, first.iterations());
    const ContactForces struck = first.pressures();
    bool any = false;
    for (const auto& [id, impulse] : struck) any = any || impulse.normal > 0;
    if (!any) return;
    bodies_ = before;
    ContactSolver solver(bodies_, touching, up(), restBounceSpeed(), &struck, &joints_);
    solver.solveImpacts();
    report_.solverIterations = std::max(report_.solverIterations, solver.iterations());
}

// Силы на отрезке h: тяжесть и реакции опоры. Они находятся как изменение импульсов за весь
// отрезок и возвращаются скоростью этого изменения; сами импульсы остаются прежними. Так
// отрезок можно оборвать в момент удара, приложив ровно ту долю сил, которая успела подействовать.
//
// Первый проход находит силы по скоростям начала отрезка. Дальше они уточняются:
//  — если тела вращаются: ускорения точек касания без сил берутся по импульсам середины
//    отрезка, которые дал первый проход (forcesPass);
//  — если есть сочленения: связь подбирается так, чтобы быть выполненной в конце отрезка точно
//    (shakeBonds).
ExactStep::Forces ExactStep::computeForces(const std::vector<ContactPair>& pairs, Real h) {
    const std::vector<Body> before = bodies_;
    Forces forces{std::vector<Rate>(bodies_.size()), {h}, std::vector<Rate>(bodies_.size())};
    BondShake shake;
    for (int pass = 0;; ++pass) {
        bool again = forcesPass(pairs, h, pass, before, shake.bias, forces);
        if (!joints_.empty()) {
            if (!shakeBonds(shake, before, forces, pass, h)) break;
            again = again || !forces.resolved;
        }
        if (!again) break;
    }
    return forces;
}

// Тела с импульсами середины отрезка при нынешних силах.
std::vector<Body> ExactStep::middleOf(const std::vector<Body>& before, const Forces& forces, Real h) {
    std::vector<Body> bodies = before;
    for (size_t i = 0; i < bodies.size(); ++i) {
        bodies[i].linearMomentum += (h / 2) * forces.rates[i].force;
        bodies[i].angularMomentum += (h / 2) * forces.rates[i].torque;
    }
    return bodies;
}

// Один проход решателя: силы — изменение импульсов за отрезок, делённое на h; тела возвращаются к
// началу отрезка. true — тела вращаются заметно, и силы нужно уточнить по импульсам середины.
bool ExactStep::forcesPass(const std::vector<ContactPair>& pairs, Real h, int pass, const std::vector<Body>& before, const std::vector<Real>& bias, Forces& forces) {
    ContactSolver solver(bodies_, pairs, up(), restBounceSpeed(), &contactForces_, &joints_); // запоминает скорости до толчка
    solver.settle(stopped_);
    if (pass > 0) solver.setDrift(middleOf(before, forces, h));
    solver.biasBonds(bias);
    for (Body& body : bodies_)
        if (!body.isStatic()) integrateVelocity(body, (1 / body.mass.inverseMass) * gravity_, {}, h);
    for (const Joint& joint : joints_) joint.applyDrive(bodies_, h);
    contactForces_ = solver.solveRestingContacts(h, contactForces_, shockPropagation);
    forces.slidingStop = solver.slidingStop(h, stoppedThisStep_);
    report_.solverIterations = std::max(report_.solverIterations, solver.iterations());
    std::vector<Vector3> bondLinear, bondAngular;
    solver.bondImpulses(bondLinear, bondAngular);
    for (size_t i = 0; i < bodies_.size(); ++i) {
        forces.rates[i] = {(1 / h) * (bodies_[i].linearMomentum - before[i].linearMomentum), (1 / h) * (bodies_[i].angularMomentum - before[i].angularMomentum)};
        forces.bonds[i] = {(1 / h) * bondLinear[i], (1 / h) * bondAngular[i]};
        bodies_[i] = before[i];
    }
    return pass == 0 && solver.setDrift(middleOf(before, forces, h)) * h >= kDriftResolved;
}

// Сила связи постоянна на отрезке и направлена как в его начале, а тела за отрезок
// поворачиваются, и связь, выполненная «по касательной», в конце отрезка расходится. Поэтому
// нарушение C в конце отрезка гасится поправкой −2 C / h к конечной скорости вдоль связи
// (перемещение при постоянной силе — средняя скорость на h). Это схема SHAKE: она симметрична во
// времени и, в отличие от сдвига тел на место без изменения скоростей, не накачивает энергию.
// Ryckaert, Ciccotti, Berendsen, «Numerical integration of the Cartesian equations of motion of a
// system with constraints», J. Comput. Phys. 23, 1977. false — подбор не сходится, отрезок
// слишком длинен (его укоротит forcesWithin).
bool ExactStep::shakeBonds(BondShake& shake, const std::vector<Body>& before, Forces& forces, int pass, Real h) const {
    std::vector<Body> end = before;
    for (size_t i = 0; i < end.size(); ++i) end[i] = advanced(before[i], forces.rates[i].force, forces.rates[i].torque, h);
    shake.errors.clear();
    for (const Joint& joint : joints_)
        for (const Bond& bond : joint.bonds(end, end)) shake.errors.push_back(bond.error);
    shake.bias.resize(shake.errors.size(), 0);
    Real error = 0, along = 0, squared = 0;
    for (size_t k = 0; k < shake.errors.size(); ++k) {
        error = std::max(error, abs(shake.errors[k]));
        if (k < shake.previousErrors.size()) along += shake.errors[k] * shake.previousErrors[k], squared += shake.previousErrors[k] * shake.previousErrors[k];
    }
    forces.resolved = error <= kBondResolved;
    // Со второго прохода оба нарушения получены при одних и тех же уточнённых ускорениях, и их
    // можно сравнивать. Поправка «−2 C / h» верна по касательной; тело поворачивается, и настоящий
    // отклик отличается от касательного в одно и то же число раз. Это число видно по тому, какая
    // доля прошлого нарушения осталась, и поправка делится на него (ускорение Эйткена). Если
    // нарушение всё равно не убывает хотя бы вдвое за проход, отрезок слишком длинен.
    if (pass >= 2 && squared > 0) shake.relaxation = std::min(Real(1.5), std::max(Real(0.25), shake.relaxation / (1 - along / squared)));
    if (!forces.resolved && (pass == kMaxBondPasses || (pass >= 3 && error > shake.previousError / 2))) return false;
    for (size_t k = 0; k < shake.errors.size(); ++k) shake.bias[k] -= shake.relaxation * 2 * shake.errors[k] / h;
    shake.previousError = error, shake.previousErrors = shake.errors;
    return true;
}

// Силы на отрезке не длиннее time. Если связи сочленений на таком отрезке не сошлись, отрезок
// делится пополам: на коротком отрезке тела поворачиваются меньше, и подбор сходится.
ExactStep::Forces ExactStep::forcesWithin(const std::vector<ContactPair>& pairs, Real& time) {
    for (;; time /= 2) {
        Forces forces = computeForces(pairs, time);
        if (forces.resolved || time + time / 2 == time) return forces;
    }
}

// Единичный вектор против силы тяжести; нулевой, если тяжести нет.
Vector3 ExactStep::up() const {
    const Real g = gravity_.length();
    return g > 0 ? (-1 / g) * gravity_ : Vector3{};
}

// Скорость, с которой тело под действием тяжести поднялось бы ровно на ширину полосы касания:
// v² = 2 g s. Удар слабее этого считается приходом в покой.
Real ExactStep::restBounceSpeed() const { return sqrt(2 * gravity_.length() * kContactDistance); }

} // namespace kp
