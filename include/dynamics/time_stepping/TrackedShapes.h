#pragma once
// Формы пары тел и их ближайшие точки в нынешних позах: по ним контакт круглого тела заново находит
// нормаль и точку внутри шага (SoftContact::follow).

#include <body/Body.h>
#include <collision/narrowphase/Gjk.h>
#include <dynamics/time_stepping/SolverBody.h>

namespace kp {

class TrackedShapes {
public:
    // Запоминает формы частей и их позы относительно центров масс тел. У двух шаров слежение не
    // нужно: там нормаль точна и по линии центров.
    void track(const Body& partA, const Body& partB);
    bool tracking() const { return shapeA_ != nullptr; }
    // Ближайшие точки форм, когда тела сдвинулись и повернулись на a и b с момента track; apart — от
    // центра масс a к центру масс b в тот момент. Координаты — от центра масс a в момент track.
    ClosestPoints closest(const BodyMotion& a, const BodyMotion& b, Vector3 apart) const;

private:
    const ConvexShape* shapeA_ = nullptr;
    const ConvexShape* shapeB_ = nullptr;
    Transform fromA_, fromB_;
};

} // namespace kp
