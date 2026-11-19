#pragma once
// Перебор активного набора односторонних линий системы: какие точки контакта и пределы хода
// давят, а какие расходятся.

#include <joints/Bond.h>
#include <dynamics/event_driven/ContactSystem.h>

#include <vector>

namespace kp {

class ActiveSet {
public:
    // pressing — набор по строкам решателя, bondPressing — по связям; оба уточняются на месте.
    ActiveSet(const std::vector<Body>& bodies, const ContactSystem& system, const std::vector<Bond>& bonds, std::vector<char>& pressing, std::vector<char>& bondPressing)
        : bodies_(bodies), system_(system), bonds_(bonds), pressing_(pressing), bondPressing_(bondPressing) {}
    // unknown — какие линии участвуют; free — скорость вдоль линии без искомых сил; force — ответ.
    void solve(const std::vector<char>& unknown, const std::vector<Real>& free, std::vector<Real>& force);

private:
    bool oneSided(size_t i) const;
    char& pressing(size_t i);
    void solveOn(std::vector<Real>& force);
    bool permute(std::vector<Real>& force);
    void descend(std::vector<Real>& force);

    const std::vector<Body>& bodies_;
    const ContactSystem& system_;
    const std::vector<Bond>& bonds_;
    std::vector<char>& pressing_;
    std::vector<char>& bondPressing_;
    // Одно решение: неизвестные и свободные члены, односторонние неизвестные, запас скорости недавящих.
    const std::vector<char>* unknown_ = nullptr;
    const std::vector<Real>* free_ = nullptr;
    std::vector<size_t> sided_;
    std::vector<Real> margin_;
};

} // namespace kp
