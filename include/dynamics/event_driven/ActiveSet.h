#pragma once
// Перебор активного набора односторонних линий системы: какие точки контакта и пределы хода
// давят, а какие расходятся.

#include <math/Span.h>
#include <math/StepMemory.h>
#include <joints/Bond.h>
#include <dynamics/event_driven/ContactSystem.h>

#include <vector>

namespace kp {

class ActiveSet {
public:
    // pressing — набор по строкам решателя, bondPressing — по связям; оба уточняются на месте.
    ActiveSet(Span<Body> bodies, const ContactSystem& system, const std::vector<Bond>& bonds, StepArray<char>& pressing, StepArray<char>& bondPressing)
        : bodies_(bodies), system_(system), bonds_(bonds), pressing_(pressing), bondPressing_(bondPressing) {}
    // unknown — какие линии участвуют; free — скорость вдоль линии без искомых сил; force — ответ.
    void solve(const StepArray<char>& unknown, const StepArray<Real>& free, StepArray<Real>& force);
    // Предел работы для шага с постоянной ценой кадра; ноль — без предела (точный шаг). Перестановкам
    // даётся factorizations разложений; не сошлись — спуск идёт от допустимой точки start (прежние силы,
    // у односторонних линий неотрицательные) и получает ещё вдвое больше. Спуск на каждом шаге остаётся в
    // допустимых силах и уменьшает функцию задачи, поэтому оборванный спуск даёт ответ не хуже прежнего —
    // в отличие от оборванных перестановок, чей промежуточный набор сил может быть каким угодно.
    int factorizations = 0;
    const StepArray<Real>* start = nullptr;
    int spent() const { return spent_; } // разложений в последнем решении

private:
    bool oneSided(size_t i) const;
    char& pressing(size_t i);
    void solveOn(StepArray<Real>& force);
    int spent_ = 0; // разложений в нынешнем решении
    StepArray<Real> descentStart();
    bool permute(StepArray<Real>& force);
    void descend(StepArray<Real>& force);

    Span<Body> bodies_;
    const ContactSystem& system_;
    const std::vector<Bond>& bonds_;
    StepArray<char>& pressing_;
    StepArray<char>& bondPressing_;
    // Одно решение: неизвестные и свободные члены, односторонние неизвестные, запас скорости недавящих.
    const StepArray<char>* unknown_ = nullptr;
    const StepArray<Real>* free_ = nullptr;
    StepArray<size_t> sided_;
    StepArray<Real> margin_;
};

} // namespace kp
