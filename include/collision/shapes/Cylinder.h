#pragma once

#include <collision/shapes/ConvexShape.h>

namespace kp {

// Самое острое ребро, какое бывает у цилиндра: радиус скругления кромки не меньше этого. Острая
// кромка — фигура с ядром во весь цилиндр: ядро перекрывается с другим телом при первом же касании,
// и расстояние между телами приходится считать для цилиндра с кромкой, скруглённой «по глубине», а
// его нижнюю оценку — для острого. У кромки они расходились на 26 мкм при допуске перекрытия
// 10 мкм, и поиск начала перекрытия шёл микросекундными шагами: шаг считался до 72 с. С одним
// постоянным скруглением, большим двух допусков, фигура одна во всех расчётах. Цена — кромка
// «острого» цилиндра скруглена на три сотых миллиметра.
constexpr Real kSharpestEdge = Real(3.2e-5); // м

// Цилиндр вдоль оси Y. rounding — радиус скругления рёбер; меньше kSharpestEdge он не бывает.
// Ядро — цилиндр, уменьшенный на rounding и по радиусу, и по высоте; вместе с радиусом скругления
// оно даёт цилиндр заданных размеров. Скругление — свойство детали, а не запас алгоритма: у
// настоящих деталей рёбра скруглены, и у такого цилиндра глубина проникновения известна точно,
// пока она меньше rounding.
class Cylinder final : public ConvexShape {
public:
    Cylinder(Real halfHeight, Real radius_, Real rounding = 0) {
        // У цилиндра мельче самого скругления оно занимает не больше четверти меньшего размера.
        const Real least = kSharpestEdge < (halfHeight < radius_ ? halfHeight : radius_) / 4 ? kSharpestEdge : (halfHeight < radius_ ? halfHeight : radius_) / 4;
        radius = rounding > least ? rounding : least;
        coreHalfHeight = halfHeight - radius, coreRadius = radius_ - radius;
    }
    // Самая далёкая точка — на окружности торца, в сторону направления.
    Vector3 support(Vector3 direction) const override {
        const Real y = direction.y < 0 ? -coreHalfHeight : coreHalfHeight;
        const Real planar = sqrt(direction.x * direction.x + direction.z * direction.z);
        if (planar == 0) return {0, y, 0};
        return {coreRadius * direction.x / planar, y, coreRadius * direction.z / planar};
    }
    Real boundingRadius() const override { return sqrt(coreRadius * coreRadius + coreHalfHeight * coreHalfHeight) + radius; }

    std::optional<Flat> flatTowards(const Transform& pose, Vector3 towards) const override;
    Outline outlineOver(const Transform& pose, const Flat& flat) const override;
    Real rollingRadius(const Transform& pose, Vector3 towards) const override;
    std::optional<Tube> tubeTowards(const Transform& pose, Vector3 towards) const override;

    Real coreHalfHeight, coreRadius;
};

} // namespace kp
