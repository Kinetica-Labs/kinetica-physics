#pragma once
// Контакт пары тел в шаге с постоянной ценой кадра: нормали в точках с мягкой поправкой положения
// и трение в центре точек — сила в касательной плоскости и момент вокруг нормали.

#include <kp/collision/ContactPair.h>
#include <kp/math/Matrix2x2.h>
#include <kp/dynamics/ContactForces.h>
#include <kp/dynamics/time_stepping/Softness.h>
#include <kp/dynamics/time_stepping/SolverBody.h>

#include <vector>

namespace kp {

class SoftContact {
public:
    // Плечи, массы и центр трения считаются один раз на кадр; накопленные импульсы — из memory.
    SoftContact(const ContactPair& pair, const std::vector<Body>& bodies, const std::vector<SolverBody>& states, const Softness& soft, const ContactForces& memory, Real h, bool fourPoints);
    // Контакт идёт за телами внутри шага мира (вызывается после каждого перемещения тел). Нормаль и
    // касательные поворачиваются с телом, которому принадлежит грань контакта; у пары рёбер — со средним
    // поворотом двух тел; у двух круглых тел нормаль — по линии их ядер. Плечо точки поворачивается со
    // своим телом; у круглого тела с телом идёт ядро, а точка касания — на радиус от него вдоль нормали.
    // С контактом, замороженным на шаг мира, сила опоры качающегося тела шла по устаревшей нормали и
    // толкала его вдоль движения: цепь из торов, отведённая на 45°, за 30 с набирала 54 → 141 Дж.
    void follow(const std::vector<SolverBody>& states);
    void warmStart(std::vector<SolverBody>& states) const;
    // Нормальные импульсы всех точек — одной задачей дополнительности (math/Complementarity.h), а не
    // по точке: проход по точкам оставляет ошибку в качании площадки, и тяжёлое тело на лёгком
    // раскачивается. Мягкость входит заменой μᵢ = λ'ᵢ / sᵢ (sᵢ — massScale точки, у упреждающей 1):
    // μ ≥ 0, w = K μ + (v − K λ + bias / s) ≥ 0, μᵢ wᵢ = 0, λ' = s μ. С одной точкой это ответ Box3D
    // (b3PushContacts): Δλ = −Kᵢᵢ⁻¹ (massScale v + bias) − impulseScale λ, massScale + impulseScale = 1.
    // Мягкость только на диагонали K была бы ошибкой: соседние точки толкали бы жёстко, со скоростью
    // pushSpeed / massScale.
    // Нынешний зазор точки — по смещению и повороту тел с начала кадра.
    void solveNormals(std::vector<SolverBody>& states, bool withBias, Real h, Real pushSpeed);
    // Момент вокруг нормали в пределах μ Σ λ r, сила в пределах μ Σ λ.
    void solveFriction(std::vector<SolverBody>& states);
    // Трение качения: момент в касательной плоскости против относительного вращения тел, не больше
    // arm · Σ λ; arm — плечо трения качения δ, м. Момент δ N не зависит от скорости, поэтому катящееся
    // тело останавливается за конечное время: шар радиуса r на полу тормозит с ускорением 5 δ g / (7 r).
    void solveRolling(std::vector<SolverBody>& states, Real arm);
    // Отскок: точка, которая до шага сближалась быстрее bounceSpeed и приняла удар, расходится со
    // скоростью e · (скорость сближения).
    void bounce(std::vector<SolverBody>& states, Real bounceSpeed);
    void remember(ContactForces& memory, Real h) const;

private:
    // Плечи r × n, повороты I⁻¹ (r × n) и матрица отклика K точек — в рабочие массивы потока.
    void respond(const SolverBody& a, const SolverBody& b) const;

    struct Point {
        ContactId key;
        Vector3 rA, rB;     // плечи точки от центров масс тел, нынешние
        Vector3 coreA, coreB; // плечи ядер в начале кадра: точка за вычетом радиуса скругления вдоль нормали
        Real base;          // зазор в начале кадра; нынешний — base + n · (расхождение точек тел с начала кадра)
        Real normalMass, lever;
        Real approach;      // нормальная скорость до шага: по ней считается отскок
        Real impulse = 0, largest = 0;
    };
    int a_, b_;
    std::vector<Point> points_;
    Vector3 normal_, tangent1_, tangent2_, centreA_, centreB_; // нынешние
    Vector3 normal0_, tangent10_, tangent20_, centreCoreA_, centreCoreB_; // начала кадра
    Vector3 apart_;  // от центра масс a к центру масс b в начале кадра
    Real roundA_ = 0, roundB_ = 0;
    bool faceOnB_ = false, edges_ = false;
    Real friction_ = 0, restitution_ = 0;
    Softness soft_;
    Matrix2x2 tangentMass_; // обратная матрица отклика трения в центре
    Real twistMass_ = 0;
    Vector2 rub_; // накопленный импульс трения вдоль двух касательных
    Real twistImpulse_ = 0;
    Matrix2x2 rollMass_; // обратная матрица отклика относительного вращения вокруг двух касательных
    Vector2 roll_;       // накопленный момент трения качения вокруг них
};

} // namespace kp
