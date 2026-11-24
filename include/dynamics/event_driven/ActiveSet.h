#pragma once
// Перебор активного набора односторонних линий системы: какие точки контакта и пределы хода
// давят, а какие расходятся.

#include <math/Span.h>
#include <math/StepMemory.h>
#include <joints/Bond.h>
#include <dynamics/event_driven/ContactSystem.h>

#include <vector>

namespace kp {

class SetFactor;

class ActiveSet {
public:
    // pressing — набор по строкам решателя, bondPressing — по связям; оба уточняются на месте.
    ActiveSet(Span<Body> bodies, const ContactSystem& system, const std::vector<Bond>& bonds, StepArray<char>& pressing, StepArray<char>& bondPressing)
        : bodies_(bodies), system_(system), bonds_(bonds), pressing_(pressing), bondPressing_(bondPressing) {}
    // unknown — какие линии участвуют; free — скорость вдоль линии без искомых сил; force — ответ.
    void solve(const StepArray<char>& unknown, const StepArray<Real>& free, StepArray<Real>& force);
    // Шаг с постоянной ценой кадра задаёт допустимую точку start (прежние силы, у односторонних линий
    // неотрицательные) и предел работы iterations — решений системы на наборе за одно solve; ноль — без
    // предела. Тогда перебор — сразу спуск от start: он на каждом шаге остаётся в допустимых силах и
    // уменьшает функцию задачи, поэтому оборванный спуск даёт ответ не хуже прежнего. У перестановок
    // промежуточный набор сил может быть каким угодно, и обрывать их нельзя.
    const StepArray<Real>* start = nullptr;
    int iterations = 0;
    int spent() const { return spent_; } // решений системы в последнем solve

private:
    bool oneSided(size_t i) const;
    char& pressing(size_t i);
    void solveOn(StepArray<Real>& force);
    void solveKept(SetFactor& factor, StepArray<char>& member, StepArray<Real>& force);
    void measure(const StepArray<Real>& force);
    int spent_ = 0; // решений системы в нынешнем solve
    StepArray<Real> descentStart();
    bool permute(StepArray<Real>& force);
    void descend(StepArray<Real>& force);
    // Вход точек в набор при спуске: lines — вошедшие в прошлый круг, refused — те, что войти не смогли,
    // together — вводить все нарушенные разом, а не одну.
    struct Entering {
        StepArray<size_t> lines;
        StepArray<char> refused;
        bool together;
    };
    bool settle(SetFactor& factor, StepArray<char>& member, StepArray<Real>& held, StepArray<Real>& force);
    bool admit(Entering& entering);

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
