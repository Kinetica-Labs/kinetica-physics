#pragma once
// Нормальные импульсы всех точек контакта — одной задачей дополнительности, а не пара за парой.
//
// Обход пар передаёт нагрузку вдоль цепочки тел по одному звену за проход и при отношении масс
// 1000 : 1 не сходится; мягкий контакт к тому же пружина с жёсткостью m ω², где m — масса самой пары,
// а не груза: под тонной куб в килограмм вдавливался на 14 см (замер H39). Здесь условия жёсткие и
// решаются разом для всех точек — тем же перебором активного набора, что в точном шаге (ActiveSet):
//   λ ≥ 0,  v_n ≥ цель,  λ (v_n − цель) = 0,
// цель — −gap / h у точки, которая ещё не коснулась (сближаться можно не быстрее, чем до касания),
// скорость выталкивания biasRate · глубина (не больше pushSpeed) у вошедшей, ноль в проходе без
// поправки положения. Ответ не зависит от отношения масс.
//
// Задача решается не на всех точках островка, а на рабочем наборе: в него входят точки с накопленным
// импульсом и самые нарушенные из остальных; прочие только проверяются по скорости, и набор
// расширяется, пока нарушенных нет. На выходе это решение полной задачи — у точки вне набора λ = 0 и
// v_n ≥ цель, — а цена растёт с числом давящих точек, а не всех: у составных тел на пару приходятся
// сотни точек при шести независимых условиях, и перебор на всех точках блуждал (три вложенных ведра —
// 57 мс на кадр).
//
// Тела делятся на островки — группы, связанные контактами через подвижные тела. Островок, в котором
// точек больше rowLimit, решается по-старому, парами на пружинах (SoftContact::solveNormals): там шаг
// приближённый. Признак — по островку целиком: если часть его точек решать жёстко, а часть на
// пружинах, он не успокаивается (шары в ведре).

#include <math/StepMemory.h>
#include <dynamics/time_stepping/ContactLevels.h>
#include <dynamics/time_stepping/SoftContact.h>

#include <vector>

namespace kp {

class IslandNormals {
public:
    IslandNormals(int rowLimit, Real biasRate) : rowLimit_(rowLimit), biasRate_(biasRate) {}
    // moving — по телам: решаются только контакты и островки, где движется хоть одно тело; nullptr — все.
    void solve(StepArray<SoftContact>& contacts, const std::vector<Body>& bodies, StepArray<SolverBody>& states, bool withBias, Real h, Real pushSpeed,
               const StepArray<char>* moving = nullptr);
    // Уровни зависимости контактов шага: ими же идут проходы тёплого старта и трения (SoftStep).
    ContactLevels levels;

private:
    // Точка контакта в задаче островка; a и b — места её тел в островке.
    struct Row {
        size_t contact, point;
        int a, b;
    };
    // Островок решается своей системой, со своей нумерацией тел: цена решения зависит от его размера,
    // а не от размера мира, и островки идут в разных потоках. Одной системой на всех цена росла как
    // куб числа тел: 400 кубов столбиками по четыре стоили 800 мс на кадр (замер study-scale).
    struct Island {
        StepArray<Row> rows;      // в порядке контактов
        StepArray<int> members;   // номера тел мира
        StepArray<Body> bodies;     // их масса, инерция и нынешняя ориентация — всё, что берёт решатель
        StepArray<char> working;  // по rows: рабочий набор
    };
    struct Pass {
        bool withBias;
        Real h, pushSpeed;
    };
    // Какие контакты идут в общую задачу: островки, уложившиеся в предел.
    void choose(const StepArray<SoftContact>& contacts, const std::vector<Body>& bodies);
    Real excess(const SoftContact::NormalRow& row, const StepArray<SolverBody>& states, const Pass& pass) const;
    void solveIsland(Island& island, StepArray<SoftContact>& contacts, const std::vector<Body>& bodies, StepArray<SolverBody>& states, const Pass& pass) const;
    void solveWorking(Island& island, StepArray<SoftContact>& contacts, StepArray<SolverBody>& states, const Pass& pass) const;
    bool extend(Island& island, const StepArray<SoftContact>& contacts, const StepArray<SolverBody>& states, const Pass& pass) const;

    int rowLimit_;
    Real biasRate_;
    StepArray<char> direct_; // по контактам; пусто — ещё не выбрано
    StepArray<Island> islands_;
};

} // namespace kp
