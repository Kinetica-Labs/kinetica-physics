#pragma once
// Луч через сцену: первое тело на его пути. Так выбирают тело под курсором, меряют расстояние
// датчиком, проверяют видимость.

#include <body/Body.h>
#include <math/Ray.h>

#include <limits>
#include <vector>

namespace kp {

struct RayHit {
    int body = -1; // номер тела; −1 — луч ни во что не попал
    int part = 0;  // номер части (Body::part)
    Real distance = 0;
    Vector3 point, normal; // точка входа и нормаль поверхности в ней, мир
    explicit operator bool() const { return body >= 0; }
};

// Ближайшее к началу луча пересечение с поверхностями тел. Начало луча внутри тела — попадание
// на нулевом расстоянии.
RayHit castRay(const std::vector<Body>& bodies, const Ray& ray);

} // namespace kp
