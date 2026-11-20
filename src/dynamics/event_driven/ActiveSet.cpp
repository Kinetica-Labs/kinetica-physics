#include <math/StepMemory.h>
#include <dynamics/event_driven/ActiveSet.h>

#include <math/ProfileMatrix.h>
#include <dynamics/event_driven/BodySpaceSolve.h>

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
    margin_.assign(system_.lines.size(), 0);
    if (!permute(force)) descend(force);
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
    for (size_t i : sided_) {
        if (pressing(i)) continue;
        margin_[i] = (*free_)[i];
        for (const ContactSystem::Entry& entry : system_.matrix[i])
            if (position[entry.column] < m) margin_[i] += entry.value * x[position[entry.column]];
    }
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

// Надёжный путь — спуск. held — нынешняя допустимая точка: силы односторонних линий в ней
// неотрицательны. Начинается он с набора, на котором остановились перестановки: первый же шаг от
// нуля выбрасывает из набора все точки с отрицательной силой.
void ActiveSet::descend(StepArray<Real>& force) {
    const StepArray<char>& unknown = *unknown_;
    const size_t total = system_.lines.size();
    StepArray<Real> held(total, 0);
    StepArray<char> refused(total, 0); // точки, которые войти в набор не смогли
    size_t entering = total;
    for (size_t round = 0; round < 3 * sided_.size() + 50; ++round) {
        for (;;) {
            solveOn(force);
            Real step = 1;
            bool blocked = false;
            for (size_t i : sided_) {
                if (!pressing(i) || force[i] > 0) continue;
                blocked = true;
                step = std::min(step, held[i] > 0 ? held[i] / (held[i] - force[i]) : Real(0));
            }
            if (!blocked) break;
            for (size_t i = 0; i < total; ++i)
                if (unknown[i]) held[i] += step * (force[i] - held[i]);
            for (size_t i : sided_)
                if (pressing(i) && force[i] <= 0 && held[i] <= kSolved / system_.diagonal[i] * Real(1e-6)) pressing(i) = 0, held[i] = 0;
        }
        held = force;
        if (entering < total) {
            if (pressing(entering)) std::fill(refused.begin(), refused.end(), 0);
            else refused[entering] = 1;
        }
        entering = total;
        Real most = -kSolved;
        for (size_t i : sided_)
            if (!pressing(i) && !refused[i] && margin_[i] < most) most = margin_[i], entering = i;
        if (entering == total) break;
        pressing(entering) = 1;
    }
}

} // namespace kp
