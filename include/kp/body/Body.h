#pragma once
// Твёрдое тело. Состояние — поза и импульсы, а не поза и скорости.
//
// Импульс p и момент импульса L — это то, что сохраняется: без внешних сил они не меняются
// вовсе, и в таком представлении не меняются побитово. Скорости выводятся из них:
//   v = p / m,   ω = R I⁻¹ Rᵀ L.
// Угловая скорость свободного тела сама меняется вместе с ориентацией — гироскопическая
// прецессия получается из формулы, отдельного гироскопического члена нет.
//
// Оси тела — главные оси инерции, начало — центр масс, поэтому тензор инерции диагонален.
//
// Невыпуклое тело — составное: несколько выпуклых частей, жёстко скреплённых в одно тело
// (CompoundBodyBuilder). Динамика у него одна, а контакт ищется для каждой части отдельно: часть
// выдаётся как тело с той же динамикой и своей формой (Body::part), и всё, что умеет работать с
// выпуклым телом, работает с частью без изменений.

#include <kp/collision/shapes/Capsule.h>
#include <kp/collision/shapes/Cylinder.h>
#include <kp/collision/shapes/Sphere.h>
#include <kp/body/MassProperties.h>
#include <kp/math/Matrix3x3.h>
#include <kp/collision/shapes/ConvexHull.h>
#include <kp/collision/broadphase/DynamicTree.h>
#include <kp/math/Transform.h>

#include <memory>
#include <utility>
#include <vector>

namespace kp {

struct Body {
    // Часть составного тела: выпуклая форма и её поза в осях тела.
    struct Part {
        std::shared_ptr<const ConvexShape> shape;
        Transform placement;
    };

    Vector3 position;       // центр масс, м
    Quaternion orientation;    // оси тела -> мир
    Vector3 linearMomentum;       // p, кг·м/с
    Vector3 angularMomentum; // L относительно центра масс, в мировых осях, кг·м²/с
    MassProperties mass;
    std::shared_ptr<const ConvexShape> shape; // в осях тела; без формы тело ни с чем не сталкивается
    // Поза формы в осях тела, если форма стоит не в них (placed): так выглядит часть составного
    // тела. У простого тела форма стоит в осях тела.
    Transform placement;
    bool placed = false;
    // Части составного тела; пусто — тело из одной формы. У составного тела shape — охватывающий
    // шар с центром в центре масс: всё, что рассуждает о теле целиком (дерево брусков, границы
    // скоростей), видит его, а контакт ищется по частям.
    std::shared_ptr<const std::vector<Part>> parts;
    // Дерево брусков частей в осях тела — у составного тела, в котором частей много: пол из
    // тысяч треугольников иначе пришлось бы перебирать подряд для каждого соседа.
    std::shared_ptr<const DynamicTree> partTree;
    Real friction = Real(0.5);    // коэффициент трения Кулона
    Real restitution = 0;         // коэффициент восстановления: 0 — неупругий удар, 1 — упругий

    // Однородный брусок с полуразмерами halfExtents. Нулевая плотность — неподвижное тело.
    static Body box(Vector3 halfExtents, Real density, Vector3 position = {}) {
        Body body;
        body.position = position;
        body.mass = MassProperties::box(halfExtents, density);
        body.shape = std::make_shared<ConvexHull>(ConvexHull::box(halfExtents));
        return body;
    }
    // Однородный шар.
    static Body sphere(Real radius, Real density, Vector3 position = {}) {
        Body body;
        body.position = position;
        body.mass = MassProperties::sphere(radius, density);
        body.shape = std::make_shared<Sphere>(radius);
        return body;
    }

    // Однородная капсула вдоль своей оси Y: halfLength — половина длины прямой части.
    static Body capsule(Real halfLength, Real radius, Real density, Vector3 position = {}) {
        Body body;
        body.position = position;
        body.mass = MassProperties::capsule(halfLength, radius, density);
        body.shape = std::make_shared<Capsule>(halfLength, radius);
        return body;
    }

    // Однородный цилиндр вдоль своей оси Y; rounding — радиус скругления рёбер (Cylinder).
    static Body cylinder(Real halfHeight, Real radius, Real density, Vector3 position = {}, Real rounding = 0) {
        Body body;
        body.position = position;
        body.mass = MassProperties::cylinder(halfHeight, radius, density);
        body.shape = std::make_shared<Cylinder>(halfHeight, radius, rounding);
        return body;
    }

    // Однородное тело в форме выпуклой оболочки облака точек (body/MassFrame.h). Точки заданы в любых
    // удобных осях; position — куда поставить начало этих осей. Центр масс и главные оси инерции
    // находятся сами, поэтому position тела и его orientation после создания — не position и не
    // единичный поворот: это центр масс и главные оси в мире.
    static Body hull(const std::vector<Vector3>& points, Real density, Vector3 position = {});

    int partCount() const { return parts ? int(parts->size()) : 1; }
    // Часть как тело: динамика та же, форма и её поза — части. У простого тела — оно само.
    Body part(int index) const {
        if (!parts) return *this;
        Body piece = *this;
        piece.shape = (*parts)[size_t(index)].shape, piece.placement = (*parts)[size_t(index)].placement, piece.placed = true;
        piece.parts = nullptr;
        return piece;
    }

    bool isStatic() const { return mass.inverseMass == 0; }
    // Поза формы в мире: у простого тела она же — оси тела.
    Transform pose() const {
        if (!placed) return {position, orientation};
        return {position + orientation.rotate(placement.position), orientation * placement.orientation};
    }
    // Оси тела в мире: начало — центр масс.
    Transform frame() const { return {position, orientation}; }
    // Радиус шара с центром в центре масс, охватывающего форму.
    Real boundingRadius() const { return (placed ? placement.position.length() : Real(0)) + shape->boundingRadius(); }

    Vector3 linearVelocity() const { return mass.inverseMass * linearMomentum; }
    // ω при заданной ориентации: L переводится в оси тела, делится на моменты, возвращается в мир.
    Vector3 angularVelocityAt(Quaternion q) const { return q.rotate(mass.inverseInertia.scaled(q.rotateInverse(angularMomentum))); }
    Vector3 angularVelocity() const { return angularVelocityAt(orientation); }

    // I⁻¹ в мировых осях, применённый к вектору: R diag(1/I) Rᵀ v.
    Vector3 inverseInertiaTimes(Vector3 v) const { return orientation.rotate(mass.inverseInertia.scaled(orientation.rotateInverse(v))); }
    // Обратный тензор инерции в мире целой матрицей: решателю с проходами он нужен тысячи раз за кадр.
    Matrix3x3 inverseInertiaInWorld() const { return Matrix3x3::fromColumns(inverseInertiaTimes({1, 0, 0}), inverseInertiaTimes({0, 1, 0}), inverseInertiaTimes({0, 0, 1})); }
    // Скорость точки тела, заданной в мире: v + ω × r.
    Vector3 pointVelocity(Vector3 point) const { return linearVelocity() + angularVelocity().cross(point - position); }
    // Импульс J, приложенный в точке мира: p += J, L += r × J.
    void applyImpulse(Vector3 impulse, Vector3 point) {
        if (isStatic()) return;
        linearMomentum += impulse;
        angularMomentum += (point - position).cross(impulse);
    }

    // Кинетическая энергия: p²/2m + ½ L·ω.
    Real kineticEnergy() const {
        return 0.5 * mass.inverseMass * linearMomentum.dot(linearMomentum) + 0.5 * angularMomentum.dot(angularVelocity());
    }
};

} // namespace kp
