#pragma once
// Память кандидатов в пары: пока ни одно тело не вышло из бруска, с которым кандидаты искались в прошлый
// раз, дерево брусков не строится и не опрашивается заново — берётся прежний список (в Box3D и
// ReactPhysics3D для того же дерево держится постоянным, а бруски в нём шире нужного).
//
// Брусок тела при постройке шире нужного на kSlack. С тех пор любая точка тела сместилась не больше чем на
// drift = |Δp| + 2 sin(θ/2) R, где R — охватывающий радиус от центра масс, а путь досягаемости стал reach
// вместо reach₀. Пока drift + reach − reach₀ <= kSlack у каждого тела, нынешний брусок каждого лежит внутри
// прежнего, и прежние кандидаты содержат всех нынешних. Лишнего кандидата отвергает проверка пары
// (examinePair), поэтому список пар от этой памяти не зависит.

#include <body/Body.h>
#include <math/StepMemory.h>

#include <utility>
#include <vector>

namespace kp {

class CandidateMemory {
public:
    static constexpr Real kSlack = Real(1e-3); // м: на столько брусок тела при постройке шире нужного
    using Pair = std::pair<int, int>;
    // Прежние кандидаты, если они годятся телам в нынешних позах с путями досягаемости reach; иначе nullptr.
    const std::vector<Pair>* valid(const std::vector<Body>& bodies, const StepArray<Real>& reach) const;
    // Новые кандидаты: они найдены по брускам тел в нынешних позах, расширенным на reach + kSlack.
    void replace(const std::vector<Body>& bodies, const StepArray<Real>& reach, const StepArray<Pair>& candidates);

private:
    struct Origin {
        Vector3 position;
        Quaternion orientation;
        const ConvexShape* shape; // у тела с другой формой брусок другой
        Real reach;
    };
    std::vector<Origin> origin_; // между шагами не освобождается
    std::vector<Pair> candidates_;
};

} // namespace kp
