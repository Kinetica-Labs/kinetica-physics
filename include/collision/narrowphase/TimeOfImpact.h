#pragma once
// Время удара консервативным продвижением.
//
// Если тела разделены расстоянием d и сближаются не быстрее чем со скоростью c, то раньше чем
// через d / c они не столкнутся. Значит, на время d / c можно продвинуться безопасно, измерить
// расстояние заново и повторить. Шаги убывают вместе с расстоянием, и продвижение сходится к
// моменту касания с безопасной стороны: тела ни в один момент не оказываются перекрытыми.
//
// Mirtich, «Impulse-based Dynamic Simulation of Rigid Body Systems», PhD thesis, UC Berkeley,
// 1996 (раздел о conservative advancement).

#include <math/math_functions.h>

namespace kp {

// distanceAt(t) — расстояние между телами в момент t от начала интервала; closingSpeed — верхняя
// граница скорости сближения на всём интервале; touching — расстояние, на котором тела считаются
// соприкоснувшимися. Возвращает момент, когда расстояние стало не больше touching, или horizon,
// если за интервал этого не произошло. Продвижение целится в середину полосы касания, чтобы
// остановиться внутри неё, а не на самой границе.
//
// Числа шагов сверху не ограничено, но оно конечно: пока расстояние больше touching, каждый шаг
// длиннее touching / 2c, так что их не больше 2c · horizon / touching. Для тел, которые летят
// вплотную друг к другу и при этом быстро вращаются, это миллионы шагов: расстояние остаётся
// малым, а граница скорости — большой. Поэтому у продвижения есть терпение: если patience шагов
// не хватило, оно возвращает horizon, а в *reached записывает момент, до которого дошло, — дальше
// интервал проходит другой поиск (EventSearch.cpp, crossingStart). patience = 0 — терпение бесконечно.
template <class DistanceAt>
Real timeOfImpact(DistanceAt&& distanceAt, Real closingSpeed, Real horizon, Real touching, int patience = 0, Real* reached = nullptr) {
    Real time = 0;
    for (int step = 0; patience == 0 || step < patience; ++step) {
        const Real distance = distanceAt(time);
        if (distance <= touching) return time;
        if (closingSpeed <= 0) return horizon;
        const Real next = time + (distance - touching / 2) / closingSpeed;
        if (next >= horizon) return horizon;
        if (next == time) return time; // шаг неразличим на часах: момент безопасный
        time = next;
    }
    *reached = time;
    return horizon;
}

} // namespace kp
