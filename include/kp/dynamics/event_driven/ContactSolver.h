#pragma once
// Решатель контактов: импульсы, после которых тела не сближаются в точках контакта.
//
// В каждой точке действуют нормальный импульс λn >= 0 и импульс трения λt, |λt| <= μ λn
// (закон Кулона: конус, а не пирамида — предел трения не зависит от направления). Импульсы
// прикладываются к двум телам с противоположным знаком в одной общей точке, поэтому суммарный
// импульс и момент импульса пары не меняются вовсе.
//
// Нормальные импульсы находятся точно, для всей системы разом. Условия во всех точках —
//   vn >= цель,  λn >= 0,  λn (vn − цель) = 0
// — это линейная задача дополнительности с матрицей A = J M⁻¹ Jᵀ. Она решается перебором
// активного набора: предполагается, в каких точках тела давят друг на друга, для них решается
// линейная система, и предположение правится там, где вышла отрицательная сила или сближение.
// Число шагов не зависит от отношения масс — в отличие от обхода «точка за точкой», которому
// при отношении 1 : 1000 не хватает тысячи проходов (`docs/experiments/30-solver-experiment.md`).
// Júdice, Pires, «A block principal pivoting algorithm for large-scale strictly monotone linear
// complementarity problems», Computers & Operations Research 21(5), 1994.
//
// Трение зависит от нормальных сил нелинейно (круг радиуса μ λn), поэтому оно решается обходом
// точек по очереди и чередуется с точным решением нормалей, пока оба не перестанут меняться.
// Пары тел упорядочены по высоте, обход трения идёт снизу вверх и обратно.

#include <kp/collision/ContactPair.h>
#include <kp/joints/JointSet.h>
#include <kp/dynamics/ContactForces.h>
#include <kp/dynamics/event_driven/ContactSystem.h>

#include <array>
#include <vector>

namespace kp {
class ContactSolver {
public:
    // up — единичный вектор против силы тяжести (нулевой, если её нет): по нему упорядочены пары.
    // pressure — силы давления в точках контакта, известные заранее (с прошлого отрезка или из
    // первого прохода): трение пары прикладывается в центре давления. Когда тело перекатывается
    // через вершину, в полосе касания две вершины, а вся нагрузка — на одной; трение, приложенное
    // посередине между ними, держало бы тело за точку, на которую оно не опирается.
    //
    // joints — сочленения: их связи решаются вместе с контактами, одной системой. Связь — равенство:
    // при ударе скорость вдоль неё становится нулевой, на отрезке опоры она не даёт связи
    // разойтись, при раздвигании возвращает разошедшуюся связь на место. Предел хода — связь
    // односторонняя, как нормаль контакта, и входит в тот же перебор активного набора. Привод с
    // конечной силой — равенство, пока его импульс не вышел за предел; вышедший прикладывается
    // как известная сила, и равенство снимается (так же срывается трение покоя).
    ContactSolver(std::vector<Body>& bodies, const std::vector<ContactPair>& pairs, Vector3 up, Real restBounceSpeed,
                  const ContactForces* pressure = nullptr, const JointSet* joints = nullptr);

    // Удар: мгновенно, без внешних сил. В касающихся точках скорость сближения v меняется на −e v.
    void solveImpacts();

    // Опора на интервале h: касающиеся точки не должны сблизиться. Сила на интервале постоянна,
    // поэтому перемещение равно средней из скоростей в начале и в конце, умноженной на h; условие
    // ставится на скорость в конце:
    //   (v₀ + v₁)/2 · h >= −s   <=>   v₁ >= −v₀ − 2 s / h,
    // где v₀ — нормальная скорость при создании решателя, до действия сил, s — зазор (у касающихся
    // точек нулевой).
    // Точки, которые ещё не коснулись, сила опоры не удерживает: их ведёт поиск времени удара
    // (EventSearch::earliestImpact), и в момент касания решается удар. Удерживать их условием на
    // перемещение нельзя — оно разворачивает быстро сближающуюся точку и добавляет телу энергию
    // (замер: validation/studies/DropStatistics.cpp).
    //
    // previous — силы с прошлого отрезка: с них решение начинается (тёплый старт). Если прошлые
    // силы на новой геометрии добавили бы кинетическую энергию, они уменьшаются в α раз, где α —
    // наилучший масштаб вдоль них. Возвращает найденные силы.
    //
    // shockPropagation — необязательный завершающий проход «ударной волны» (Guendelman, Bridson,
    // Fedkiw, «Nonconvex rigid bodies with stacking», 2003). Пары обходятся снизу вверх, и каждая
    // решается заново так, будто нижнее тело неподвижно; поправка прикладывается только к
    // верхнему. Остаток, оставленный обходом трения, гасится за один проход — ценой третьего
    // закона Ньютона: нижнее тело не получает противодействия, и импульс системы не сохраняется.
    // Поэтому по умолчанию проход выключен.
    ContactForces solveRestingContacts(Real h, const ContactForces& previous, bool shockPropagation = false);

    // Поправки к целям связей на отрезке опоры: на сколько скорость вдоль каждой связи в конце
    // отрезка должна отличаться от найденной по касательной оценке. Ими World доводит связи до
    // точного выполнения в конце отрезка (ExactStep::computeForces).
    void biasBonds(const std::vector<Real>& bias) { bondBias_ = bias; }

    // Доля связей сочленений в найденных импульсах: что они дали каждому телу. Приводы, трение,
    // пружины и демпферы не считаются: это силы, а не связи, и толкают тело весь отрезок (World,
    // advanced).
    void bondImpulses(std::vector<Vector3>& linear, std::vector<Vector3>& angular) const;

    int iterations() const { return iterations_; }

    // Нормальные импульсы в касающихся точках после решения — как давление для следующего прохода.
    ContactForces pressures() const;

    // Пересчитывает ускорения точек касания без сил по скоростям тел moving; возвращает наибольшее
    // из них, м/с².
    Real setDrift(const std::vector<Body>& moving);

    // Раздвигание тел, вошедших друг в друга. Решатель вызывается для «теней» тел — копий с
    // нулевыми импульсами — и находит импульсы, после которых каждая перекрывшаяся точка за
    // единицу времени разойдётся ровно на глубину перекрытия. Импульс тени, проинтегрированный за
    // эту единицу времени, даёт сдвиг позы; настоящие импульсы тел не меняются, поэтому тела
    // расходятся, не получая скорости. Это то, что в других движках называют псевдоскоростями
    // (split impulse в Bullet и ReactPhysics3D), только без отдельных полей у тела: псевдоскорость —
    // это скорость тени.
    void solveOverlap();

    // Когда скольжение какой-нибудь точки остановится. Вызывается после solveRestingContacts(h):
    // скорость точки на отрезке меняется линейно от начальной до найденной конечной, и если по
    // дороге она разворачивается, скольжение прекратилось в момент наименьшей скорости.
    // Возвращает этот момент — наименьший по всем парам — и саму пару, или h, если разворотов нет.
    // Остановка скольжения — событие того же рода, что удар: отрезок обрывается на нём, и
    // следующий начинается с пары, которая уже не скользит.
    //
    // Разворот бывает двух родов. Если скольжение по дороге проходит через ноль — пара
    // остановилась (halted). Если оно только повернуло — костяшка домино скользит по соседней, и
    // направление скольжения уходит в сторону, — пара продолжает скользить: отрезок обрывается, чтобы
    // трение следующего было направлено против нового скольжения, но остановившейся пара не
    // считается. Пока поворот принимался за остановку, такая пара объявлялась неподвижной, тут
    // же срывалась и снова «останавливалась» — до трёх тысяч событий за шаг.
    struct SlidingStop {
        Real time = 0;
        int a = -1, b = -1;
        bool halted = false;
    };
    //
    // already — пары, у которых такое событие в этом шаге мира уже было: второй раз за шаг их
    // разворот событием не считается. Пара на грани срыва меняет направление скольжения снова и
    // снова, и без этого правила шаг дробится без конца — как тело, приходящее в покой, отскакивало
    // бы бесконечно много раз (ср. restBounceSpeed). Цена — направление трения такой пары до
    // конца шага может отставать от её скольжения; энергии оно при этом не добавляет
    // (frictionImpulse).
    SlidingStop slidingStop(Real h, const std::vector<std::pair<int, int>>& already = {}) const;

    // Пара, остановившаяся в конце прошлого отрезка, не скользит с начала этого — по определению
    // события, а не по измеренной скорости. Силы оборванного отрезка пересчитываются на его
    // длину и останавливают пару не точно; без этого правила остаток скольжения давал бы ту же
    // остановку снова и снова, через миллиардные доли секунды, и шаг не кончался бы.
    void settle(SlidingStop stopped);

private:
    struct Row {
        ContactId key;
        Vector3 point, normal;
        Real separation, restitution;
        Real normalMass;   // 1 / (n · K n): сколько импульса даёт единицу скорости
        Real startSpeed;   // нормальная скорость при создании решателя
        Real drift;        // ускорение сближения точек двух тел без всяких сил, м/с²: см. geometricAcceleration
        Real roundA, roundB; // радиусы качения тел в месте контакта (ContactManifold)
        bool faceOnB;      // грань контакта принадлежит телу b
        bool touching;     // точка в полосе касания
    };
    using Slip = std::array<Real, 3>;                    // вдоль двух касательных и вокруг нормали
    using Response = std::array<std::array<Real, 3>, 3>; // матрица отклика трения пары
    // Строки одной пары тел и её трение. Трение пары приложено в одной точке — центре её точек
    // касания: сила в касательной плоскости и момент вокруг нормали, который мешает одному телу
    // вращаться по другому. Так же устроено трение в ReactPhysics3D
    // (ContactSolverSystem.cpp:445, «twist friction constraint at the center of the contact
    // manifold»). Неизвестных три — столько же, сколько у пары степеней свободы в плоскости
    // касания. Трение в каждой точке отдельно дало бы восемь неизвестных на те же три степени:
    // решение неоднозначно, и обход точек по очереди не сходится.
    struct Pair {
        size_t first, last; // строки: [first, last)
        int a, b;
        Vector3 normal, tangent1, tangent2, centre;
        Real friction;
        Real roundA, roundB; // радиусы качения тел в месте контакта (ContactManifold)
        Real reach;         // среднее расстояние точек от центра, м: переводит вращение вокруг нормали в скорость
        Response inverse;   // обратная матрица отклика: какой импульс гасит единичное скольжение
        Slip response;      // диагональ матрицы отклика: скольжение от единичного импульса
        Slip startSlip;     // скольжение при создании решателя, до действия сил
        Slip drift;         // ускорение скольжения без всяких сил: то же, что Row::drift, для трения
        Slip impulse;       // накопленный импульс трения: сила, сила, момент
    };
    using Line = ContactSystem::Line;
    using Entry = ContactSystem::Entry;
    using System = ContactSystem;

    // goal[i] — наименьшая допустимая нормальная скорость в точке i; минус бесконечность — точка
    // в этой фазе не участвует. Накопленные импульсы — в sum_.
    void solve(const std::vector<Real>& goal);
    System assemble(const std::vector<Real>& goal);
    void addFriction(System& system, const std::vector<Real>& goal);
    void addBonds(System& system);
    Real residual(const System& system, const std::vector<Real>& goal) const;
    Real solveTogether(const System& system, const std::vector<Real>& goal);
    // Части solveTogether: начальные импульсы и цели, неизвестные, свободные члены, срыв пар
    // по Кулону, упор приводов, приложение ответа.
    void startTogether(const System& system, const std::vector<Real>& goal, std::vector<Real>& current, std::vector<Real>& target) const;
    std::vector<char> unknownsOf(const System& system) const;
    std::vector<Real> freeTerms(const System& system, const std::vector<char>& unknown, const std::vector<Real>& current, const std::vector<Real>& target) const;
    bool releaseSlipping(const System& system, const std::vector<Real>& force);
    bool releaseDrives(const System& system, const std::vector<char>& unknown, std::vector<Real>& current, const std::vector<Real>& target, const std::vector<Real>& force);
    Real applyTogether(const System& system, const std::vector<Real>& force);
    std::vector<size_t> eliminationOrder(const System& system) const;
    Line normalLine(const Row& row) const;
    Line frictionLine(const Pair& pair, int k) const;
    static Line bondLine(const Bond& bond) { return {bond.a, bond.b, bond.linear, bond.angularA, bond.angularB}; }
    void applyLine(const Line& line, Real impulse);
    Real speed(const Line& line) const;
    Real solveFriction(Pair& pair, const std::vector<Real>& goal);
    Pair frictionOf(const ContactPair& contact, size_t first, size_t last) const;
    Response frictionResponse(const Pair& pair, bool withA, bool withB) const;
    Slip frictionImpulse(const Pair& pair, const std::vector<Real>& goal, const Response& inverse) const;
    Slip slip(const Pair& pair) const;
    void applyFriction(const Pair& pair, const Slip& impulse, bool toA, bool toB);
    void reset();
    void warmStart(const std::vector<Real>& goal, const ContactForces& previous, Real h);
    void propagateShock(const std::vector<Real>& goal);
    Real relativeSpeed(const Row& row, Vector3 direction) const;
    void apply(const Row& row, Vector3 impulse);
    Real kineticEnergy() const;

    std::vector<Body>& bodies_;
    const ContactForces* pressure_;
    Vector3 up_;
    std::vector<Row> rows_;
    std::vector<Pair> pairs_;
    std::vector<Real> sum_;      // накопленные нормальные импульсы строк
    std::vector<char> pressing_; // активный набор: точки, где тела давят друг на друга
    std::vector<char> stuck_;    // пары, которые не скользят: их трение решается вместе с нормалями
    Real restBounceSpeed_;
    const JointSet* joints_;
    std::vector<Bond> bonds_;         // связи всех сочленений подряд
    std::vector<Real> bondImpulse_;   // накопленные импульсы связей
    std::vector<Real> bondStart_;     // скорость вдоль связи при создании решателя
    std::vector<Real> bondGoal_;      // скорость вдоль связи, которой добивается нынешняя фаза
    std::vector<Real> bondBias_;      // поправки к целям на отрезке опоры, см. biasBonds
    std::vector<char> bondActive_;    // связь участвует в нынешнем решении
    std::vector<char> bondPressing_;  // активный набор пределов хода: сочленение давит на предел
    std::vector<signed char> saturated_; // привод упёрся в предел силы: знак его импульса; 0 — не упёрся
    std::vector<int> bondFlips_;      // сколько раз привод упирался и отпускался: перебор обязан кончиться
    Real driveCap(size_t bond) const; // наибольший импульс привода в нынешней фазе
    bool holding_ = false; // решается опора на отрезке, а не мгновенный удар
    Real holdTime_ = 0;    // длина отрезка, на котором решается опора; при ударе ноль
    int iterations_ = 0;
};

} // namespace kp
