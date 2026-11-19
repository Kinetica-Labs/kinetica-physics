#pragma once
// Масса, центр масс и главные оси инерции однородного выпуклого многогранника.
//
// Тело в движке задано в своих главных осях с началом в центре масс: там тензор инерции
// диагонален (Body.h). У бруска и шара эти оси известны заранее; у произвольной оболочки их нужно
// найти. Интегралы берутся разбиением на тетраэдры с вершиной в начале координат, главные оси —
// собственные векторы тензора инерции.

#include <body/MassProperties.h>
#include <collision/shapes/ConvexHull.h>
#include <math/Transform.h>

namespace kp {

struct MassFrame {
    MassProperties mass;
    Transform frame;   // оси тела в осях, в которых задана оболочка: начало — центр масс, оси — главные
    Real volume;  // м³

    static MassFrame of(const ConvexHull& hull, Real density);
};

} // namespace kp
