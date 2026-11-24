#include <math/StepMemory.h>
#include <dynamics/event_driven/ActiveSet.h>

#include <math/ProfileMatrix.h>
#include <dynamics/event_driven/BodySpaceSolve.h>
#include <dynamics/event_driven/SetFactor.h>

#include <algorithm>

namespace kp {

// Перебор активного набора односторонних линий — нормалей точек контакта и пределов хода
// сочленений — при заданных неизвестных. Линии трения и запертых свобод участвуют всегда: это
// равенства, знак их импульса не ограничен.
//
// Быстрый путь — перестановка всех нарушений сразу (Júdice, Pires): нарушения — отрицательная
// сила в давящей точке или сближение в недавящей — исправляются разом, пока их число убывает.
// Обычно хватает нескольких решений системы.
//
// Надёжный путь — когда число нарушений убывать перестало. Так бывает в вырожденных задачах: в
// пирамиде кубов сотни точек касаются без нагрузки, сила и скорость в них нулевые одновременно, и
// перестановки блуждают. Задача — наименьшее значение выпуклой функции ½ λᵀ A λ + bᵀ λ при λ >= 0
// на односторонних линиях, и она решается спуском по активному набору (Lawson, Hanson, «Solving
// least squares problems», 1974, глава 23): в набор вводится точка с наибольшим сближением;
// если решение на новом наборе делает чью-то силу отрицательной, шаг к нему укорачивается до
// первой обнулившейся силы, и эта точка из набора выходит. Функция убывает на каждом шаге, поэтому
// наборы не повторяются и перебор конечен.
void ActiveSet::solve(const StepArray<char>& unknown, const StepArray<Real>& free, StepArray<Real>& force) {
    unknown_ = &unknown, free_ = &free;
    sided_.clear();
    for (size_t i = 0; i < system_.lines.size(); ++i)
        if (unknown[i] && oneSided(i)) sided_.push_back(i);
    margin_.assign(system_.lines.size(), 0), spent_ = 0;
    if (start || !permute(force)) descend(force);
}

bool ActiveSet::oneSided(size_t i) const {
    const size_t n = system_.rows.size(), firstBond = system_.lines.size() - bonds_.size();
    return i < n || (i >= firstBond && bonds_[i - firstBond].oneSided);
}

char& ActiveSet::pressing(size_t i) {
    const size_t n = system_.rows.size(), firstBond = system_.lines.size() - bonds_.size();
    return i < n ? pressing_[system_.rows[i]] : bondPressing_[i - firstBond];
}

// Решение системы на нынешнем наборе: силы участвующих линий — в force, запас скорости недавящих
// односторонних — в margin_. Сначала разложение в осях тел (BodySpaceSolve); если оно дороже
// профиля — Холецкий по профилю.
void ActiveSet::solveOn(StepArray<Real>& force) {
    ++spent_;
    const StepMemory::Scope scope; // force и margin_ созданы раньше и здесь не растут
    const StepArray<char>& unknown = *unknown_;
    const size_t total = system_.lines.size();
    StepArray<size_t> active;
    for (size_t i : system_.order)
        if (unknown[i] && (!oneSided(i) || pressing(i))) active.push_back(i);
    const size_t m = active.size();
    StepArray<size_t> position(total, m); // место линии среди участвующих; m — не участвует
    for (size_t p = 0; p < m; ++p) position[active[p]] = p;
    StepArray<size_t> leftmost(m);
    Real cost = 0; // цена разложения: сумма квадратов ширины профиля
    for (size_t p = 0; p < m; ++p) {
        leftmost[p] = p;
        for (const ContactSystem::Entry& entry : system_.matrix[active[p]]) leftmost[p] = std::min(leftmost[p], position[entry.column]);
        cost += Real(p - leftmost[p] + 1) * Real(p - leftmost[p] + 1);
    }
    StepArray<Real> x(m);
    for (size_t p = 0; p < m; ++p) x[p] = -(*free_)[active[p]];
    if (!BodySpaceSolve(bodies_, system_).solve(active, cost, x)) {
        ProfileMatrix a;
        for (size_t p = 0; p < m; ++p) {
            a.addRow(int(leftmost[p]));
            for (const ContactSystem::Entry& entry : system_.matrix[active[p]])
                if (position[entry.column] <= p) a.at(int(p), int(position[entry.column])) = entry.value;
            a.at(int(p), int(p)) *= 1 + kProximal;
            a.at(int(p), int(p)) += system_.soft[active[p]]; // у жёсткой линии ноль: биты прежние
        }
        solveCholesky(a, x);
    }
    std::fill(force.begin(), force.end(), Real(0));
    for (size_t p = 0; p < m; ++p) force[active[p]] = x[p];
    measure(force);
}

// Запас скорости недавящих односторонних линий при силах force (у линий вне набора они нулевые).
void ActiveSet::measure(const StepArray<Real>& force) {
    for (size_t i : sided_) {
        if (pressing(i)) continue;
        margin_[i] = (*free_)[i];
        for (const ContactSystem::Entry& entry : system_.matrix[i]) margin_[i] += entry.value * force[entry.column];
    }
}

// То же решение на нынешнем наборе, но разложением, идущим за набором (SetFactor): в спуске набор между
// соседними решениями отличается на одну-две линии, и правка стоит m² действий, а не разложения заново.
void ActiveSet::solveKept(SetFactor& factor, StepArray<char>& member, StepArray<Real>& force) {
    ++spent_;
    for (size_t i = 0; i < member.size(); ++i) member[i] = (*unknown_)[i] && (!oneSided(i) || pressing(i));
    factor.follow(member), factor.solve(*free_, force);
    measure(force);
}

// Быстрый путь: все нарушения переставляются разом, пока их число убывает (не дольше десяти
// решений без убыли). true — набор найден.
bool ActiveSet::permute(StepArray<Real>& force) {
    int fewest = int(sided_.size()) + 1, patience = 10;
    for (;;) {
        const StepMemory::Scope scope;
        solveOn(force);
        StepArray<size_t> violated;
        for (size_t i : sided_) {
            // Допуск на силу — тот же допуск на скорость, переведённый в импульс: иначе точка с
            // силой −10⁻¹⁵ считалась бы нарушением, а после перестановки нарушением считалось бы
            // её сближение, и она переставлялась бы туда и обратно.
            if (pressing(i) ? force[i] < -kSolved / system_.diagonal[i] : margin_[i] < -kSolved) violated.push_back(i);
        }
        if (violated.empty()) return true;
        if (int(violated.size()) < fewest) fewest = int(violated.size()), patience = 10;
        else if (patience-- == 0) return false;
        for (size_t i : violated) pressing(i) = !pressing(i);
    }
}

// Точка, с которой начинается спуск: ноль или заданная допустимая точка start; набор — её давящие линии
// (набор, на котором остановились перестановки, к ней отношения не имеет).
StepArray<Real> ActiveSet::descentStart() {
    StepArray<Real> held(system_.lines.size(), 0);
    if (!start) return held;
    held = *start;
    for (size_t i : sided_) pressing(i) = held[i] > 0;
    return held;
}

// Надёжный путь — спуск. held — нынешняя допустимая точка: силы односторонних линий в ней
// неотрицательны. Без start он начинается с набора, на котором остановились перестановки: первый же шаг
// от нуля выбрасывает из набора все точки с отрицательной силой.
void ActiveSet::descend(StepArray<Real>& force) {
    StepArray<Real> held = descentStart();
    StepArray<char> member(system_.lines.size());
    SetFactor factor(system_);
    Entering entering{{}, StepArray<char>(system_.lines.size(), 0), true};
    for (size_t round = 0; round < 3 * sided_.size() + 50; ++round) {
        if (!settle(factor, member, held, force)) return void(force = held); // предел работы: допустимая точка
        held = force;
        if (!admit(entering)) break;
    }
}

// Шаг от held к решению на нынешнем наборе. Если оно делает чью-то силу отрицательной, шаг укорачивается
// до первой обнулившейся силы, эта точка из набора выходит, и решение повторяется. false — предел работы
// исчерпан; held при этом допустима.
bool ActiveSet::settle(SetFactor& factor, StepArray<char>& member, StepArray<Real>& held, StepArray<Real>& force) {
    for (;;) {
        if (iterations > 0 && spent_ >= iterations) return false;
        solveKept(factor, member, force);
        Real step = 1;
        bool blocked = false;
        for (size_t i : sided_) {
            if (!pressing(i) || force[i] > 0) continue;
            blocked = true;
            step = std::min(step, held[i] > 0 ? held[i] / (held[i] - force[i]) : Real(0));
        }
        if (!blocked) return true;
        for (size_t i = 0; i < held.size(); ++i)
            if ((*unknown_)[i]) held[i] += step * (force[i] - held[i]);
        for (size_t i : sided_)
            if (pressing(i) && force[i] <= 0 && held[i] <= kSolved / system_.diagonal[i] * Real(1e-6)) pressing(i) = 0, held[i] = 0;
    }
}

// В набор входят все нарушенные точки разом: на холодном старте и при обвале кучи их десятки, и по одной
// предел работы кончался бы раньше набора. Шаг к решению на расширенном наборе функцию не увеличивает —
// прежняя точка лежит в том же подпространстве, — но вошедшие разом могут все получить отрицательную силу
// и выйти, не сдвинув точку. Тогда следующий круг вводит одну, самую нарушенную: её сила на новом наборе
// положительна (Lawson, Hanson), и функция убывает строго. Точка, не вошедшая и одна, откладывается до
// первого удачного входа. false — нарушенных нет: набор найден.
bool ActiveSet::admit(Entering& entering) {
    const bool stayed = std::any_of(entering.lines.begin(), entering.lines.end(), [&](size_t i) { return pressing(i) != 0; });
    if (stayed) std::fill(entering.refused.begin(), entering.refused.end(), 0);
    else if (entering.lines.size() == 1) entering.refused[entering.lines[0]] = 1;
    entering.together = stayed || entering.lines.empty() || (entering.together && entering.lines.size() == 1);
    entering.lines.clear();
    size_t worst = system_.lines.size();
    Real most = -kSolved;
    for (size_t i : sided_) {
        if (pressing(i) || entering.refused[i] || !(margin_[i] < -kSolved)) continue;
        if (entering.together) entering.lines.push_back(i);
        if (margin_[i] < most) most = margin_[i], worst = i;
    }
    if (worst == system_.lines.size()) return false;
    if (!entering.together) entering.lines.push_back(worst);
    for (size_t i : entering.lines) pressing(i) = 1;
    return true;
}

} // namespace kp
