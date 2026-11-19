#pragma once
// Преобразование: где находится тело и как повёрнуто. Переводит точки и направления из осей
// тела в мир и обратно.

#include <math/Quaternion.h>

namespace kp {

struct Transform {
    Vector3 position;
    Quaternion orientation;

    Vector3 point(Vector3 local) const { return position + orientation.rotate(local); }
    Vector3 direction(Vector3 local) const { return orientation.rotate(local); }
    Vector3 localDirection(Vector3 world) const { return orientation.rotateInverse(world); }
    Vector3 localPoint(Vector3 world) const { return orientation.rotateInverse(world - position); }
    // Преобразование `other` в осях этого. Положения вычитаются до поворота: разность близких
    // больших чисел точна, поэтому геометрия пары тел не теряет точность вдали от начала координат.
    Transform relative(const Transform& other) const { return {localDirection(other.position - position), orientation.conjugate() * other.orientation}; }
};

} // namespace kp
