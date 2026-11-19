#pragma once
// Масса и главные моменты инерции тела, готовые формулы для простых форм.

#include <math/Vector3.h>
#include <math/math_functions.h>

namespace kp {

// Масса и главные моменты инерции. Нулевая обратная масса — неподвижное тело.
struct MassProperties {
    Real inverseMass = 0;
    Vector3 inverseInertia; // 1/Ix, 1/Iy, 1/Iz в осях тела, 1/(кг·м²)

    // Однородный брусок с полуразмерами h: m = 8 ρ hx hy hz, Ix = m (hy² + hz²) / 3.
    static MassProperties box(Vector3 h, Real density) {
        if (density == 0) return {};
        const Real m = 8 * density * h.x * h.y * h.z;
        return {1 / m, {3 / (m * (h.y * h.y + h.z * h.z)), 3 / (m * (h.x * h.x + h.z * h.z)),
                        3 / (m * (h.x * h.x + h.y * h.y))}};
    }
    // Однородный шар: m = 4/3 π ρ r³, I = 2/5 m r².
    static MassProperties sphere(Real radius, Real density) {
        if (density == 0) return {};
        const Real m = Real(4) / 3 * kPi * density * radius * radius * radius;
        const Real inverseI = 2.5 / (m * radius * radius);
        return {1 / m, {inverseI, inverseI, inverseI}};
    }
    // Однородный цилиндр вдоль оси Y: m = 2 π ρ r² h, Iy = m r² / 2, Ix = Iz = m (3 r² + 4 h²) / 12,
    // где h — половина высоты.
    static MassProperties cylinder(Real halfHeight, Real radius, Real density) {
        if (density == 0) return {};
        const Real m = 2 * kPi * density * radius * radius * halfHeight;
        const Real across = m * (3 * radius * radius + 4 * halfHeight * halfHeight) / 12;
        return {1 / m, {1 / across, 2 / (m * radius * radius), 1 / across}};
    }
    // Однородная капсула вдоль оси Y: цилиндр с половиной длины h и два полушара радиуса r.
    // У полушаров, сдвинутых на концы: I поперёк оси = m (2 r² / 5 + h² + 3 h r / 4).
    static MassProperties capsule(Real halfLength, Real radius, Real density) {
        if (density == 0) return {};
        const Real tube = 2 * kPi * density * radius * radius * halfLength, caps = Real(4) / 3 * kPi * density * radius * radius * radius;
        const Real along = tube * radius * radius / 2 + caps * 2 * radius * radius / 5;
        const Real across = tube * (3 * radius * radius + 4 * halfLength * halfLength) / 12 +
                            caps * (2 * radius * radius / 5 + halfLength * halfLength + 3 * halfLength * radius / 4);
        return {1 / (tube + caps), {1 / across, 1 / along, 1 / across}};
    }
};

} // namespace kp
