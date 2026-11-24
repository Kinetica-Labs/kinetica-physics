#pragma once
// Внешняя зубчатая передача на двух параллельных неподвижных осях.
// Подшипники задаются двумя HingeJoint. Эта связь связывает только повороты колёс:
// ZA θA + ZB θB = 0, углы отсчитываются от поз при сборке. Зубья идеальны, без люфта.
#include <joints/Joint.h>

namespace kp {

class GearJoint final : public Joint {
public:
    // shaft — общее положительное направление осей в мире, teethA и teethB — числа зубьев.
    // Разные тела, конечная ненулевая ось и положительные числа зубьев обязательны.
    GearJoint(int a, const Body& bodyA, int b, const Body& bodyB, Vector3 shaft, int teethA, int teethB);
    std::unique_ptr<Joint> clone() const override { return std::make_unique<GearJoint>(*this); }

protected:
    void build(const Motion& motion, std::vector<Bond>& bonds) const override;

private:
    Real angle(Quaternion rotation) const;
    Vector3 shaft_;
    int turnsA_, turnsB_; // числа зубьев после сокращения на их НОД
};

} // namespace kp
