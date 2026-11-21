#include <dynamics/event_driven/SetFactor.h>

#include <algorithm>

namespace kp {

// Правка на линию стоит порядка stored() действий, разложение с нуля — во много раз больше, поэтому
// заново оно собирается редко. Линии, добавленные правками, идут в конец разложения и профиль расширяют:
// когда L хранит вдвое больше, чем после сборки, каждая правка вдвое дороже, и разложение собирается
// заново — теми же добавлениями строк, в порядке system.order, где профиль узкий. Замер на рывке куба
// пирамиды: сборка после каждых «ширина строки» правок давала кадр до 21 мс, без неё — до 14 мс.
void SetFactor::follow(const StepArray<char>& member) {
    const bool rebuilt = factor_.stored() > 2 * fresh_;
    if (rebuilt) {
        factor_.clear(), lines_.clear();
        std::fill(place_.begin(), place_.end(), place_.size());
    }
    compare(member);
    for (size_t place : leaving_) remove(place);
    for (size_t line : entering_) append(line);
    if (rebuilt || fresh_ == 0) fresh_ = factor_.stored();
}

// Выходящие — с конца разложения: хвост, который правит удаление, короче, и места остальных выходящих
// от удаления не сдвигаются.
void SetFactor::compare(const StepArray<char>& member) {
    leaving_.clear(), entering_.clear();
    for (size_t p = lines_.size(); p-- > 0;)
        if (!member[lines_[p]]) leaving_.push_back(p);
    for (size_t line : system_.order)
        if (member[line] && place_[line] == place_.size()) entering_.push_back(line);
}

// Профиль строки — от самой ранней связанной с ней линии набора. Диагональ — как в ActiveSet::solveOn:
// с добавкой kProximal и податливостью линии.
void SetFactor::append(size_t line) {
    size_t first = lines_.size();
    for (const ContactSystem::Entry& entry : system_.matrix[line]) first = std::min(first, place_[entry.column]);
    work_.assign(lines_.size() - first, 0);
    Real diagonal = 0;
    for (const ContactSystem::Entry& entry : system_.matrix[line]) {
        if (entry.column == line) diagonal = entry.value * (1 + kProximal) + system_.soft[line];
        else if (place_[entry.column] < lines_.size()) work_[place_[entry.column] - first] = entry.value;
    }
    factor_.append(int(first), work_, diagonal);
    place_[line] = lines_.size(), lines_.push_back(line);
}

void SetFactor::remove(size_t place) {
    factor_.remove(int(place));
    place_[lines_[place]] = place_.size();
    lines_.erase(lines_.begin() + std::ptrdiff_t(place));
    for (size_t p = place; p < lines_.size(); ++p) place_[lines_[p]] = p;
}

void SetFactor::solve(const StepArray<Real>& free, StepArray<Real>& force) {
    work_.resize(lines_.size());
    for (size_t p = 0; p < lines_.size(); ++p) work_[p] = -free[lines_[p]];
    factor_.solve(work_);
    std::fill(force.begin(), force.end(), Real(0));
    for (size_t p = 0; p < lines_.size(); ++p) force[lines_[p]] = work_[p];
}

} // namespace kp
