#include <kp/dynamics/time_stepping/SoftStep.h>

#include <kp/dynamics/time_stepping/DirectJoints.h>
#include <kp/dynamics/time_stepping/SoftContact.h>
#include <kp/dynamics/time_stepping/SoftJoint.h>

#include <algorithm>
#include <limits>

namespace kp {
namespace {

// Состояния тел для решателя на начало кадра; у неподвижного тела — нули.
std::vector<SolverBody> solverBodiesOf(const std::vector<Body>& bodies) {
    std::vector<SolverBody> states(bodies.size());
    for (size_t i = 0; i < bodies.size(); ++i) {
        const Body& body = bodies[i];
        if (body.isStatic()) continue;
        states[i].inverseMass = body.mass.inverseMass, states[i].v = body.linearVelocity(), states[i].w = body.angularVelocity();
        states[i].inverseInertia = body.inverseInertiaInWorld();
    }
    return states;
}

// Контакты пар; у пары с неподвижным телом мягкость своя (staticSoft), как у Box3D.
std::vector<SoftContact> contactsOf(const std::vector<ContactPair>& pairs, const std::vector<Body>& bodies, const std::vector<SolverBody>& states, const Softness& soft,
                                    const Softness& staticSoft, const ContactForces& memory, Real h, bool fourPoints) {
    std::vector<SoftContact> contacts;
    for (const ContactPair& pair : pairs) {
        const Body &bodyA = bodies[size_t(pair.a)], &bodyB = bodies[size_t(pair.b)];
        if (pair.manifold.count == 0 || (bodyA.isStatic() && bodyB.isStatic())) continue;
        contacts.emplace_back(pair, bodies, states, bodyA.isStatic() || bodyB.isStatic() ? staticSoft : soft, memory, h, fourPoints);
    }
    return contacts;
}

// Момент импульса по угловой скорости: L = R I Rᵀ ω, I — главные моменты тела.
Vector3 momentumOf(const Body& body, Vector3 w) {
    const Vector3 inertia{1 / body.mass.inverseInertia.x, 1 / body.mass.inverseInertia.y, 1 / body.mass.inverseInertia.z};
    return body.orientation.rotate(inertia.scaled(body.orientation.rotateInverse(w)));
}

// Тела сдвигаются с найденными скоростями. Момент импульса при повороте сохраняется, а угловая
// скорость пересчитывается по новой ориентации: это и есть гироскопический член. origin и facing —
// позы начала кадра: от них считаются сдвиг и поворот, по которым контакт знает нынешний зазор.
void moveBodies(std::vector<Body>& bodies, std::vector<SolverBody>& states, const std::vector<Vector3>& origin, const std::vector<Quaternion>& facing, Real h) {
    for (size_t i = 0; i < bodies.size(); ++i) {
        SolverBody& s = states[i];
        if (s.inverseMass == 0) continue;
        Body& body = bodies[i];
        const Vector3 momentum = momentumOf(body, s.w);
        body.position += h * s.v;
        body.angularMomentum = momentum;
        // У тела с неравными моментами инерции угловая скорость меняется вместе с ориентацией,
        // поэтому поворот идёт со скоростью середины подшага. Со скоростью его начала вытянутое
        // тело само раскручивалось вокруг длинной оси: на 2 % за кадр.
        const Vector3 inverse = body.mass.inverseInertia;
        const bool round = inverse.x == inverse.y && inverse.y == inverse.z;
        const Vector3 turn = round ? h * s.w : h * body.angularVelocityAt((Quaternion::fromRotationVector((h / 2) * s.w) * body.orientation).unit());
        body.orientation = (Quaternion::fromRotationVector(turn) * body.orientation).unit();
        s.moved = body.position - origin[i], s.turned = body.orientation * facing[i].conjugate();
        s.w = body.angularVelocity();
        s.inverseInertia = body.inverseInertiaInWorld();
    }
}

// Связи сочленений по нынешним позам тел: запертые свободы — одной системой (DirectJoints),
// остальные — мягкими связями по одной. Строятся заново после каждого сдвига тел: проход без
// поправки идёт уже по новому положению, и следующий подшаг начинает с тех же связей. Половина
// импульса связи приходится на старое направление, половина — на новое; если оба прохода вести по
// старому, скорость каждый подшаг срезается на косинус поворота, и маятник теряет 2,5 % энергии в секунду.
struct JointRows {
    std::vector<SoftJoint> links;
    DirectJoints locked;

    void build(const JointSet& joints, const std::vector<Body>& bodies, const std::vector<SolverBody>& states, std::vector<std::vector<Real>>& memory, bool direct) {
        links.clear(), locked.clear();
        for (size_t j = 0; j < joints.size(); ++j) {
            const std::vector<Bond> bonds = joints[j].bonds(bodies, bodies);
            if (memory[j].size() != bonds.size()) memory[j].assign(bonds.size(), 0);
            for (size_t k = 0; k < bonds.size(); ++k) {
                const Bond& bond = bonds[k];
                if (direct && !bond.soft && !bond.oneSided && (!bond.driven || bond.strength == std::numeric_limits<Real>::infinity())) {
                    locked.add(bond, &memory[j][k]);
                    continue;
                }
                const SoftJoint link(&bond, 1, states, &memory[j][k]);
                if (link.responds()) links.push_back(link);
            }
        }
        locked.assemble(states);
    }
    void warmStart(std::vector<SolverBody>& states) const {
        locked.warmStart(states);
        for (const SoftJoint& link : links) link.warmStart(states);
    }
    void solve(std::vector<SolverBody>& states, bool withBias, Real correction, Real h, const Softness& soft, Real pushSpeed) {
        locked.solve(states, withBias ? correction : 0, h);
        for (SoftJoint& link : links) link.solve(states, withBias, h, soft, pushSpeed);
    }
};

} // namespace

void SoftStep::step(std::vector<Body>& bodies, const std::vector<ContactPair>& pairs, const JointSet& joints, Vector3 gravity, Real dt, ContactForces& contactMemory,
                    std::vector<std::vector<Real>>& jointMemory) const {
    const Real h = dt / substeps;
    const Real hertz = std::min(contactHertz, Real(0.125) / h);
    const Softness soft = Softness::make(hertz, contactDamping, h), staticSoft = Softness::make(2 * hertz, contactDamping / 2, h);
    const Softness jointSoft = Softness::make(std::min(jointHertz, Real(0.25) / h), jointDamping, h);

    for (const Joint& joint : joints) joint.applyDrive(bodies, dt);
    std::vector<SolverBody> states = solverBodiesOf(bodies);
    std::vector<SoftContact> contacts = contactsOf(pairs, bodies, states, soft, staticSoft, contactMemory, h, fourContactPoints);
    std::vector<Vector3> origin(bodies.size());
    std::vector<Quaternion> facing(bodies.size());
    for (size_t i = 0; i < bodies.size(); ++i) origin[i] = bodies[i].position, facing[i] = bodies[i].orientation;
    jointMemory.resize(joints.size());
    JointRows rows;
    rows.build(joints, bodies, states, jointMemory, directJoints);

    for (int substep = 0; substep < substeps; ++substep) {
        for (SolverBody& s : states)
            if (s.inverseMass > 0) s.v += h * gravity;
        // Тёплый старт: накопленные импульсы прикладываются к новым скоростям.
        rows.warmStart(states);
        for (const SoftContact& contact : contacts) contact.warmStart(states);
        rows.solve(states, true, jointCorrection, h, jointSoft, pushSpeed);
        for (SoftContact& contact : contacts) contact.solveNormals(states, true, h, pushSpeed);
        moveBodies(bodies, states, origin, facing, h);
        for (SoftContact& contact : contacts) contact.follow(states);
        rows.build(joints, bodies, states, jointMemory, directJoints);
        rows.solve(states, false, jointCorrection, h, jointSoft, pushSpeed);
        for (SoftContact& contact : contacts) contact.solveNormals(states, false, h, pushSpeed);
        for (SoftContact& contact : contacts) contact.solveFriction(states);
        for (SoftContact& contact : contacts) contact.solveRolling(states, rollingResistance);
    }

    for (SoftContact& contact : contacts) contact.bounce(states, bounceSpeed);
    for (size_t i = 0; i < bodies.size(); ++i)
        if (states[i].inverseMass != 0) bodies[i].linearMomentum = (1 / states[i].inverseMass) * states[i].v, bodies[i].angularMomentum = momentumOf(bodies[i], states[i].w);
    contactMemory.clear();
    for (const SoftContact& contact : contacts) contact.remember(contactMemory, h);
}

} // namespace kp
