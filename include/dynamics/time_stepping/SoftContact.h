#pragma once
// Контакт пары тел в шаге с постоянной ценой кадра: нормали в точках с мягкой поправкой положения
// и трение в центре точек — сила в касательной плоскости и момент вокруг нормали.

#include <math/StepMemory.h>
#include <collision/ContactPair.h>
#include <collision/narrowphase/Gjk.h>
#include <math/Matrix2x2.h>
#include <dynamics/ContactForces.h>
#include <dynamics/time_stepping/Softness.h>
#include <dynamics/time_stepping/CentreFriction.h>
#include <dynamics/time_stepping/SolverBody.h>
#include <dynamics/time_stepping/TrackedShapes.h>

#include <vector>

namespace kp {

struct BlockWork; // рабочая память решения площадки (SoftContact.cpp)

class SoftContact {
public:
    // Плечи, массы и центр трения считаются один раз на кадр; накопленные импульсы — из known, записей
    // сил этой пары с прошлого шага (ContactForces::of); у контакта, найденного посреди шага, их нет.
    // Контакт может появиться и посреди шага (MissedContacts): смещения и повороты тел он отсчитывает
    // от момента своего создания. tracked — у круглых тел ближайшие точки ищутся заново внутри шага.
    SoftContact(const ContactPair& pair, const std::vector<Body>& bodies, const StepArray<SolverBody>& states, const Softness& soft, Span<ContactForces::Entry> known, Real h, bool fourPoints,
                bool tracked);
    // Контакт идёт за телами внутри шага мира (вызывается после каждого перемещения тел). Нормаль и
    // касательные поворачиваются с телом, которому принадлежит грань контакта; у пары рёбер — со средним
    // поворотом двух тел; у двух круглых тел нормаль — по линии их ядер. Плечо точки поворачивается со
    // своим телом; у круглого тела с телом идёт ядро, а точка касания — на радиус от него вдоль нормали.
    // Точки двух тел за шаг расходятся; импульс прикладывается к обоим в одной общей точке — в точке
    // тела, лежащего на грани другого, — иначе пара импульсов ±J создаёт момент: в куче без тяжести
    // момент импульса системы уходил на 0,24 % за 5 с.
    // С контактом, замороженным на шаг мира, сила опоры качающегося тела шла по устаревшей нормали и
    // толкала его вдоль движения: цепь из торов, отведённая на 45°, за 30 с набирала 54 → 141 Дж.
    void follow(const StepArray<SolverBody>& states);
    void warmStart(StepArray<SolverBody>& states) const;
    // Нормальные импульсы всех точек — одной задачей дополнительности (math/Complementarity.h), а не
    // по точке: проход по точкам оставляет ошибку в качании площадки, и тяжёлое тело на лёгком
    // раскачивается. Мягкость входит заменой μᵢ = λ'ᵢ / sᵢ (sᵢ — massScale точки, у упреждающей 1):
    // μ ≥ 0, w = K μ + (v − K λ + bias / s) ≥ 0, μᵢ wᵢ = 0, λ' = s μ. С одной точкой это ответ Box3D
    // (b3PushContacts): Δλ = −Kᵢᵢ⁻¹ (massScale v + bias) − impulseScale λ, massScale + impulseScale = 1.
    // Мягкость только на диагонали K была бы ошибкой: соседние точки толкали бы жёстко, со скоростью
    // pushSpeed / massScale.
    // Нынешний зазор точки — по смещению и повороту тел с начала кадра.
    void solveNormals(StepArray<SolverBody>& states, bool withBias, Real h, Real pushSpeed);
    // То же, когда нормали всех контактов решаются одной задачей (IslandNormals): строка нормали
    // точки k и её новый импульс — разница прикладывается к телам.
    struct NormalRow {
        int a, b;
        Vector3 normal, rA, rB;
        Real gap, impulse;
    };
    size_t pointCount() const { return points_.size(); }
    NormalRow normalRow(size_t k) const { return {a_, b_, normal_, points_[k].rA, points_[k].rB, points_[k].gap, points_[k].impulse}; }
    void setNormalImpulse(StepArray<SolverBody>& states, size_t k, Real impulse);
    // Момент вокруг нормали в пределах μ Σ λ r, сила в пределах μ Σ λ; затем трение качения с
    // плечом rollingArm (RollingFriction).
    void solveFriction(StepArray<SolverBody>& states, Real rollingArm);
    // Для контакта, созданного по позе после пробного сдвига, когда тела возвращены в начало подшага h
    // (MissedContacts). Строка решателя линейна по скорости, а путь точки вращающегося тела — дуга:
    // при 90 рад/с линейный прогноз зазора расходится с настоящим на 2–3 мм за подшаг. Настоящий
    // зазор после сдвига контакт знает — с ним он создан; разница с прогнозом g + h v_n при тех же
    // скоростях добавляется к зазору точки.
    void matchTrial(const StepArray<SolverBody>& states, Real h);
    // Отскок: точка, которая при создании контакта сближалась быстрее bounceSpeed, коснулась и приняла
    // удар, расходится со скоростью e · (скорость сближения). Вызывается сразу после решения
    // нормалей, один раз на точку: в цепочке шаров удары идут по очереди, и отскок, посчитанный
    // в конце шага для всех разом, давал не тот ответ (маятник Ньютона).
    void bounce(StepArray<SolverBody>& states, Real bounceSpeed);
    void remember(ContactForces& memory, Real h) const;
    // Подкачка памяти впереди прохода по контактам: сам контакт far (его адрес известен заранее) и точки
    // и тела контакта near (их адреса лежат в контакте, к этому времени уже подкачанном); проход зовёт
    // её перед контактом, до которого от near kNear шагов, а от far — kFar. Номер за концом списка
    // пропускается. Аппаратная подкачка останавливается на границе страницы, и первый доступ на каждой
    // странице ждал основную память: 13,5 % тактов кадра пирамиды из 1 015 кубов. Ответ от неё не зависит.
    static constexpr size_t kNear = 4, kFar = 8; // 2 и 4, 8 и 16 дают то же
    static void prefetch(const StepArray<SoftContact>& contacts, const StepArray<SolverBody>& states, size_t near, size_t far);

private:
    friend class CoupledFriction;
    CentreFriction::Carried takePoints(const ContactPair& pair, Vector3 xA, Vector3 xB, const SolverBody& a, const SolverBody& b, Span<ContactForces::Entry> known, Real h, bool fourPoints);
    // Нынешние нормаль и касательные; found — ближайшие точки форм, найденные заново, или nullptr.
    void aim(const BodyMotion& a, const BodyMotion& b, Vector3 apart, const ClosestPoints* found);

    // Плечи r × n, повороты I⁻¹ (r × n) и матрица отклика K точек — в рабочую память work.
    void respond(const SolverBody& a, const SolverBody& b, const BlockWork& work) const;

    struct Point {
        ContactId key;
        Vector3 rA, rB;     // плечи точки от центров масс тел, нынешние
        Vector3 coreA, coreB; // плечи ядер в начале кадра: точка за вычетом радиуса скругления вдоль нормали
        Real base, gap;     // зазор при создании контакта и нынешний: base + n · (расхождение точек тел с тех пор)
        Real found;         // зазор, с которым точка найдена
        Real normalMass, lever; // normalMass и approach считаются только у пары с отскоком: их читает один bounce
        Real approach;      // нормальная скорость до шага: по ней считается отскок
        Real impulse = 0;
    };
    int a_, b_;
    StepArray<Point> points_;
    Vector3 normal_, tangent1_, tangent2_, centreA_, centreB_; // нынешние
    Vector3 normal0_, tangent10_, tangent20_, centreCoreA_, centreCoreB_; // начала кадра
    Vector3 apart_;  // от центра масс a к центру масс b в начале кадра
    BodyMotion createdA_, createdB_; // смещения и повороты тел (SolverBody) в момент создания контакта
    Real roundA_ = 0, roundB_ = 0;
    // Слежение за формами: у двух круглых тел и у круглого тела с многогранником при одной точке.
    TrackedShapes shapes_;
    bool faceOnB_ = false, edges_ = false;
    CentreFriction::Frame frame() const { return {normal_, tangent1_, tangent2_, centreA_, centreB_}; }
    Real restitution_ = 0;
    Softness soft_;
    CentreFriction friction_; // трение в центре точек — сила в касательной плоскости и момент вокруг нормали
};

} // namespace kp
