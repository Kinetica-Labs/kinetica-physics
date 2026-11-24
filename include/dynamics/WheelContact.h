#pragma once
// Силы одного нагруженного колеса в осях дороги. Вход — сжатие подвески и скорость
// пятна колеса относительно дороги (у вращающегося колеса она включает ω × r).
#include <math/math_functions.h>

namespace kp {
struct WheelSettings {
    Real stiffness, damping;                 // Н/м, Н·с/м по нормали к дороге
    Real longitudinalDrag, lateralDrag;      // Н·с/м в плоскости дороги
    Real longitudinalFriction, lateralFriction; // безразмерные коэффициенты сцепления
};

struct WheelForces {
    Real normal, longitudinal, lateral; // Н, сила дороги на колесо
};

class WheelContact {
public:
    explicit WheelContact(WheelSettings settings);
    // Сжатие в м, положительная скорость сжатия и обе скорости скольжения в м/с.
    // При отрыве (compression ≤ 0) сила равна нулю.
    WheelForces forces(Real compression, Real compressionRate, Real longitudinalSlip, Real lateralSlip) const;

private:
    WheelSettings settings_;
};
} // namespace kp
