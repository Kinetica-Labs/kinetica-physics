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

Real PathJoint::placeOf(const Body& bodyA, const Body& bodyB) const { return path_->closest(bodyA.pose().localPoint(bodyB.pose().point(anchorB))); }

Real PathJoint::coordinate(const Body& bodyA, const Body& bodyB) const { return path_->length(placeOf(bodyA, bodyB)); }

// Сила привода — вдоль касательной, к обоим телам в точке сочленения тела b.
void PathJoint::applyDrive(std::vector<Body>& bodies, Real h) const {
    if (driveForce == 0) return;
    Body &bodyA = bodies[size_t(a)], &bodyB = bodies[size_t(b)];
    const Vector3 push = (h * driveForce) * bodyA.pose().direction(path_->tangent(placeOf(bodyA, bodyB)));
    const Vector3 at = bodyB.pose().point(anchorB);
    applyImpulse(bodyB, push, at);
    applyImpulse(bodyA, -1 * push, at);
}

// Оси сочленения в ближайшей точке кривой: x — касательная, точка сочленения тела a — точка кривой.
// С ними связи собираются как у ползуна; поперечным добавляется центростремительное слагаемое.
void PathJoint::build(const Motion& m, std::vector<Bond>& bonds) const {
    const Transform seat = m.bodyA->pose();
    const Real u = path_->closest(seat.localPoint(m.pb));
    const Vector3 tangent = seat.direction(path_->tangent(u)), bend = seat.direction(path_->curvature(u));
    Motion along = m;
    along.frameA = turningXTo(tangent), along.pa = seat.point(path_->point(u));
    const Real speed = tangent.dot(m.pointB - m.pointA); // скорость точки вдоль кривой
    for (int k = 1; k < 3; ++k) {
        Bond across = travelLine(along, k);
        across.drift -= across.linear.dot(bend) * speed * speed;
        bonds.push_back(across);
    }
    Bond freedom = travelLine(along, 0);
    freedom.error = 0;
    if (lockTurns) lockAllTurns(m, bonds);
    addFreedomBonds(m, freedom, bonds);
}

} // namespace kp
