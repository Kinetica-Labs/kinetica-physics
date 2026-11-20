#pragma once
// Контакты, которых не было в площадке пары на начало обновления.
//
// Площадка контакта находится раз в обновление (SoftStep::contactUpdates) и к его концу устаревает:
// у вращающегося многогранника в соседа входит вершина, которой в площадке не было, и ни одна строка
// решателя её не держит. По 16 вариантам кучи из тридцати тел худшее перекрытие было 9,5 мм в
// медиане и до 32 мм; число подшагов его не меняло, а частота обновления меняла (замер H43).
//
// Поэтому после пробного сдвига тел пары проверяются заново в новой позе — те, чьи точки прошли за
// подшаг больше допуска. Если глубина пары за подшаг выросла больше чем на допуск (решатель вёл её
// точки к нулевому зазору, значит, вошло то, чего он не видел), по новой площадке создаётся ещё один
// контакт этой пары; шаг возвращает тела в начало подшага и решает нормали снова, уже с ним
// (SoftStep::step). Новый контакт держит именно те точки тел, что вошли друг в друга, и ближайших
// точек заново не ищет: иначе в начале подшага он совпал бы с прежним. Так же шаг без пересечений
// (IPC) проверяет сдвиг до того, как его принять; здесь проверяется поза конца подшага, а не путь.

#include <math/StepMemory.h>
#include <dynamics/time_stepping/SoftContact.h>

#include <functional>
#include <vector>

namespace kp {

class MissedContacts {
public:
    using Maker = std::function<SoftContact(const ContactPair&)>;
    // owner — номер пары каждого контакта; depth — допуск глубины, м; boundary — PairFinder::boundary.
    // Список pairs не копируется: он обязан пережить этот объект.
    MissedContacts(const std::vector<ContactPair>& pairs, const std::vector<Body>& bodies, StepArray<size_t> owner, Real depth, Real boundary);
    // Пара, найденная посреди шага (PairReach); withContact — вызывающий дописывает в contacts её контакт.
    void adopt(const ContactPair& pair, const std::vector<Body>& bodies, bool withContact);
    // Глубина каждой пары до сдвига — по точкам её контактов.
    void note(const StepArray<SoftContact>& contacts);
    // Проверяет пары в позах после сдвига и дописывает недостающие контакты (их создаёт make);
    // h — подшаг. true — дописан хотя бы один.
    bool find(StepArray<SoftContact>& contacts, const std::vector<Body>& bodies, const StepArray<SolverBody>& states, Real h, const Maker& make);

private:
    size_t pairCount() const { return listed_.size() + adopted_.size(); }
    const ContactPair& pair(size_t p) const { return p < listed_.size() ? listed_[p] : adopted_[p - listed_.size()]; }
    void measure(const ContactPair& pair, const std::vector<Body>& bodies);

    const std::vector<ContactPair>& listed_; // пары шага
    StepArray<ContactPair> adopted_;         // пары, найденные посреди шага: их номера идут следом
    StepArray<size_t> owner_;
    StepArray<Real> radiusA_, radiusB_; // охватывающие радиусы частей пары от центров масс тел
    StepArray<Real> known_;             // глубина пары до сдвига
    Real depth_, boundary_;
};

} // namespace kp
