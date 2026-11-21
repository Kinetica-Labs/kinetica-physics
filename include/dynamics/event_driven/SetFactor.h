#pragma once
// Разложение системы на наборе линий, которое идёт за набором: когда перебор (ActiveSet) вводит в
// набор линию или выводит её, разложение правится на эту линию, а не считается заново.

#include <math/StepMemory.h>
#include <math/UpdatedCholesky.h>
#include <dynamics/event_driven/ContactSystem.h>

namespace kp {

class SetFactor {
public:
    explicit SetFactor(const ContactSystem& system) : system_(system), place_(system.lines.size(), system.lines.size()) {}
    // Приводит разложение к набору member (по линиям): вышедшие линии убираются, вошедшие добавляются.
    void follow(const StepArray<char>& member);
    // Силы линий набора при правой части −free; у остальных линий — ноль.
    void solve(const StepArray<Real>& free, StepArray<Real>& force);

private:
    void compare(const StepArray<char>& member);
    void append(size_t line);
    void remove(size_t place);

    const ContactSystem& system_;
    UpdatedCholesky factor_;
    StepArray<size_t> lines_;              // линии набора в порядке разложения
    StepArray<size_t> place_;              // место линии в разложении; lines.size() — не в наборе
    StepArray<size_t> leaving_, entering_; // места выходящих линий и номера входящих
    StepArray<Real> work_;                 // строка матрицы и правая часть: массивы одного решения, без выделений в цикле
    size_t fresh_ = 0;                     // сколько чисел хранило разложение сразу после сборки с нуля
};

} // namespace kp
