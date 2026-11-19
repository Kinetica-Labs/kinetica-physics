#pragma once
// Перебор активного набора односторонних линий системы: какие точки контакта и пределы хода
// давят, а какие расходятся.

#include <math/StepMemory.h>
#include <joints/Bond.h>
#include <dynamics/event_driven/ContactSystem.h>

#include <vector>

namespace kp {

class ActiveSet {
public:
    // pressing — набор по строкам решателя, bondPressing — по связям; оба уточняются на месте.
    ActiveSet(const std::vector<Body>& bodies, const ContactSystem& system, const std::vector<Bond>& bonds, StepArray<char>& pressing, StepArray<char>& bondPressing)
        : bodies_(bodies), system_(system), bonds_(bonds), pressing_(pressing), bondPressing_(bondPressing) {}
    // unknown — какие линии участвуют; free — скорость вдоль линии без искомых сил; force — ответ.
    void solve(const StepArray<char>& unknown, const StepArray<Real>& free, StepArray<Real>& force);

private:
    bool oneSided(size_t i) const;
    char& pressing(size_t i);
    void solveOn(StepArray<Real>& force);
    bool permute(StepArray<Real>& force);
    void descend(StepArray<Real>& force);

    const std::vector<Body>& bodies_;
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
