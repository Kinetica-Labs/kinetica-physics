#include <joints/PathJoint.h>

namespace kp {
namespace {

std::vector<Vector3> localTo(const Body& body, const std::vector<Vector3>& world) {
    std::vector<Vector3> local;
    for (Vector3 point : world) local.push_back(body.pose().localPoint(point));
    return local;
}

} // namespace

PathJoint::PathJoint(int a_, const Body& bodyA, int b_, const Body& bodyB, Vector3 anchor, const std::vector<Vector3>& through, bool closed)
    : SingleFreedomJoint(a_, bodyA, b_, bodyB, anchor), path_(std::make_shared<const Path>(localTo(bodyA, through), closed)) {
    // У открытой кривой есть концы: без пределов хода тело съехало бы с неё по касательной.
    if (!closed) lower = 0, upper = path_->length();
}

// Координата — длина дуги до ближайшей точки кривой плюс смещение от неё вдоль касательной. Внутри кривой
// смещение нулевое: ближайшая точка — основание перпендикуляра. За концом открытой кривой ближайшая точка
// стоит в конце, и координата растёт дальше по касательной — предел хода видит, насколько тело ушло за конец.
PathJoint::Place PathJoint::placeOf(Vector3 local) const {
    const Real u = path_->closest(local);
    return {u, path_->length(u) + path_->tangent(u).dot(local - path_->point(u))};
}

Real PathJoint::coordinate(const Body& bodyA, const Body& bodyB) const { return placeOf(bodyA.pose().localPoint(bodyB.pose().point(anchorB))).along; }

// Сила привода — вдоль касательной, к обоим телам в точке сочленения тела b.
void PathJoint::applyDrive(std::vector<Body>& bodies, Real h) const {
    if (driveForce == 0) return;
    Body &bodyA = bodies[size_t(a)], &bodyB = bodies[size_t(b)];
    const Vector3 at = bodyB.pose().point(anchorB);
    const Vector3 push = (h * driveForce) * bodyA.pose().direction(path_->tangent(path_->closest(bodyA.pose().localPoint(at))));
    applyImpulse(bodyB, push, at);
    applyImpulse(bodyA, -1 * push, at);
}

// Точка сочленения тела a — ближайшая точка кривой; связи собираются как у ползуна с осью вдоль
// касательной, поперечным добавляется центростремительное слагаемое.
void PathJoint::build(const Motion& m, std::vector<Bond>& bonds) const {
    const Transform seat = m.bodyA->pose();
    const Place place = placeOf(seat.localPoint(m.pb));
    const Vector3 tangent = seat.direction(path_->tangent(place.u)), normal = seat.direction(path_->normal(place.u));
    const Vector3 bend = seat.direction(path_->curvature(place.u));
    Motion along = m;
    along.pa = seat.point(path_->point(place.u));
    const Real speed = tangent.dot(m.pointB - m.pointA); // скорость точки вдоль кривой
    for (Vector3 across : {normal, tangent.cross(normal)}) {
        Bond bond = travelLine(along, across);
        bond.drift -= across.dot(bend) * speed * speed;
        bonds.push_back(bond);
    }
    Bond freedom = travelLine(along, tangent);
    freedom.error = 0;
    if (lockTurns) lockAllTurns(m, bonds);
    addFreedomBonds(freedom, place.along, bonds);
}

} // namespace kp
