#pragma once
// Геометрия захвата в метрах, плотность в кг/м³, усилия в ньютонах. Ось x направлена вдоль сжатия.

namespace grip {
struct Box {
    double x, halfX, halfY, halfZ, density;
};
constexpr Box leftJaw{-0.07, 0.02, 0.45, 0.06, 500};
constexpr Box rightJaw{0.07, 0.02, 0.45, 0.06, 500};
constexpr Box cube{0, 0.05, 0.05, 0.05, 1000};
constexpr double cubeMass = 8 * cube.halfX * cube.halfY * cube.halfZ * cube.density;

struct Load {
    double friction, squeeze, down; // μ, сила одного пальца [Н], нагрузка на куб вниз [Н]
    bool open = false; // контроль датчика: при начальном зазоре 1 м губки не достигают куба
};
} // namespace grip
