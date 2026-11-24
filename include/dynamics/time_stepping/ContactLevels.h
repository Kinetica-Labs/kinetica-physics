#pragma once
// Обход контактов всеми потоками без смены ответа.
//
// Решатель обходит контакты по очереди, и ответ зависит от порядка только у контактов с общим подвижным
// телом: каждый читает скорости тела, записанные предыдущим. Уровень контакта — на единицу больше
// наибольшего уровня более раннего контакта с тем же подвижным телом. У контактов одного уровня общих
// подвижных тел нет, а зависимые идут в прежней очерёдности, поэтому счёт уровня всеми потоками даёт тот
// же ответ бит в бит, что обход по порядку. Раскраска графа этого не умеет: она порядок меняет, и
// перекрытие тел при ударах росло вдвое (журнал попыток, H74 и H79).
//
// У пирамиды из 1 015 кубов 150 уровней на 5 292 контакта. Цепочка тел даёт уровни из одного контакта:
// такую сцену, как и сцену из немногих контактов, дешевле обойти одним потоком.

#include <math/StepMemory.h>
#include <dynamics/time_stepping/SoftContact.h>

#include <functional>

namespace kp {

class ContactLevels {
public:
    // job(c) для каждого контакта. Ответ тот же, что у обхода по возрастанию c, если job пишет только в
    // свой контакт и в подвижные тела этого контакта.
    void each(const StepArray<SoftContact>& contacts, const StepArray<SolverBody>& states, const std::function<void(size_t)>& job);

private:
    void build(const StepArray<SoftContact>& contacts, const StepArray<SolverBody>& states);
    void together(const StepArray<SoftContact>& contacts, const StepArray<SolverBody>& states, const std::function<void(size_t)>& job) const;

    StepArray<size_t> order_; // контакты по уровням; внутри уровня — по возрастанию номеров
    StepArray<size_t> first_; // начало каждого уровня в order_ и конец последнего
};

} // namespace kp
