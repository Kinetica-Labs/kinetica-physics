#pragma once
// Сборка системы решателя контактов: строки давящих точек, трение покоящихся пар и связи сочленений —
// и порядок исключения неизвестных.

#include <math/StepMemory.h>
#include <dynamics/event_driven/ContactRows.h>

namespace kp {

class ContactAssembly {
public:
    explicit ContactAssembly(ContactRows& state) : state_(state) {}

    // goal[i] — наименьшая допустимая нормальная скорость в точке i; минус бесконечность — точка
    // в этой фазе не участвует.
    ContactSystem assemble(const StepArray<Real>& goal);

private:
    using Row = ContactRows::Row;
    using Pair = ContactRows::Pair;
    using Slip = ContactRows::Slip;
    using Response = ContactRows::Response;
    using Line = ContactRows::Line;
    using Entry = ContactSystem::Entry;
    using System = ContactSystem;

    void addFriction(System& system, const StepArray<Real>& goal);
    void addBonds(System& system);
    StepArray<size_t> eliminationOrder(const System& system) const;

    ContactRows& state_;
};

} // namespace kp
