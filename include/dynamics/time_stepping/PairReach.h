#pragma once
// Досягаемость тел, на которую рассчитан список пар.
//
// Поиск пар (PairFinder) берёт те, что могут сблизиться за обновление при скоростях его начала: точка
// тела проходит за время t не больше (|v| + c |L| + g t) t, где c — скорость точки на единицу момента
// импульса (maxPointSpeed). Тело, получившее удар внутри шага, движется быстрее и доходит до тела,
// пары с которым в списке нет: у маятника Ньютона покоящиеся шары входили друг в друга на 3 мм, у
// чаши ведро входило в стенку на 5,7 мм (замер H45).
//
// Здесь на каждое тело ведётся счёт пройденного пути против этой границы. Когда следующий подшаг
// вывел бы тело за неё, пары ищутся заново — при нынешних скоростях, на оставшееся время обновления,
// — и те, которых в списке не было, отдаются шагу до сдвига тел.

#include <math/StepMemory.h>
#include <collision/ContactPair.h>
#include <collision/PairFinder.h>
#include <joints/JointSet.h>

#include <array>
#include <vector>

namespace kp {

class PairReach {
public:
    // time — время обновления, на которое найден список pairs; slack — на сколько телу можно выйти
    // за границу, м: столько перекрытия останется незамеченным.
    PairReach(const std::vector<Body>& bodies, const std::vector<ContactPair>& pairs, Vector3 gravity, Real time, Real slack);
    // Путь подшага h при нынешних импульсах тел; true — какое-то тело выйдет за свою границу.
    bool exceeded(const std::vector<Body>& bodies, Real h);
    // Подшаг принят: путь, посчитанный последним exceeded, засчитан.
    void spend();
    // Пары заново, на время remaining; возвращает те, которых в списке не было. Границы всех тел
    // ставятся по нынешним скоростям. boundary — PairFinder::boundary.
    StepArray<ContactPair> discover(const std::vector<Body>& bodies, const JointSet& joints, Real remaining, Real boundary);

private:
    // Путь точки тела: linear + c · angular, c — от наименьшего до наибольшего у частей тела.
    struct Path {
        Real linear = 0, angular = 0;
    };
    using Key = PairFinder::Key;
    void allow(const std::vector<Body>& bodies, Real time);

    Vector3 gravity_;
    Real slack_;
    StepArray<Real> least_, most_; // c частей тела: наименьшее и наибольшее
    StepArray<Path> allowed_, spent_, next_;
    StepArray<Key> listed_; // пары списка, по возрастанию
};

} // namespace kp
