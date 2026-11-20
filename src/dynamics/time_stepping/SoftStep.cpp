#include <math/StepMemory.h>
#include <dynamics/time_stepping/SoftStep.h>

#include <dynamics/time_stepping/IslandNormals.h>
#include <dynamics/time_stepping/DirectJoints.h>
#include <dynamics/time_stepping/MissedContacts.h>
#include <dynamics/time_stepping/PairReach.h>
#include <dynamics/time_stepping/SoftContact.h>
#include <dynamics/time_stepping/SoftJoint.h>
#include <math/WorkerPool.h>

#include <algorithm>
#include <limits>

namespace kp {
namespace {

// Состояния тел для решателя на начало кадра; у неподвижного тела — нули.
StepArray<SolverBody> solverBodiesOf(const std::vector<Body>& bodies) {
    StepArray<SolverBody> states(bodies.size());
    for (size_t i = 0; i < bodies.size(); ++i) {
        const Body& body = bodies[i];
        if (body.isStatic()) continue;
        states[i].inverseMass = body.mass.inverseMass, states[i].v = body.linearVelocity(), states[i].w = body.angularVelocity();
        states[i].inverseInertia = inverseInertiaInWorld(body);
    }
    return states;
}

// Контакт пары; у пары с неподвижным телом мягкость своя (staticSoft), как у Box3D.
struct ContactMaker {
    const std::vector<Body>& bodies;
    const StepArray<SolverBody>& states;
    Softness soft, staticSoft;
    Real h;
    bool fourPoints;

    // tracked — у круглых тел контакт заново ищет ближайшие точки внутри шага (SoftContact::follow).
    // known — силы пары с прошлого шага; у контакта, найденного посреди шага, их нет: тёплый старт
    // его импульсов к телам не прикладывал.
    SoftContact make(const ContactPair& pair, bool tracked, Span<ContactForces::Entry> known = {}) const {
        const bool fixed = bodies[size_t(pair.a)].isStatic() || bodies[size_t(pair.b)].isStatic();
        return SoftContact(pair, bodies, states, fixed ? staticSoft : soft, known, h, fourPoints, tracked);
    }
    // Контакты пар, у которых есть точки; owner — номер пары каждого контакта.
    StepArray<SoftContact> all(const std::vector<ContactPair>& pairs, const ContactForces& memory, StepArray<size_t>& owner) const {
        StepArray<SoftContact> contacts;
        contacts.reserve(pairs.size()), owner.reserve(pairs.size());
        const ContactForces::Entry* next = memory.begin();
        for (size_t p = 0; p < pairs.size(); ++p) {
            const ContactPair& pair = pairs[p];
            if (pair.manifold.count == 0 || (bodies[size_t(pair.a)].isStatic() && bodies[size_t(pair.b)].isStatic())) continue;
            const Span<ContactForces::Entry> known = memory.of({pair.a, pair.b, 0, pair.partA, pair.partB}, next);
            next = known.end();
            contacts.push_back(make(pair, true, known)), owner.push_back(p);
        }
        return contacts;
    }
};

// Наибольшая глубина, на которую вошли точки контакта, м; ноль — ни одна не вошла.
Real deepestOf(const SoftContact& contact) {
    Real depth = 0;
    for (size_t k = 0; k < contact.pointCount(); ++k) depth = std::max(depth, -contact.normalRow(k).gap);
    return depth;
}

// Момент импульса по угловой скорости: L = R I Rᵀ ω, I — главные моменты тела.
Vector3 momentumOf(const Body& body, Vector3 w) {
    const Vector3 inertia{1 / body.mass.inverseInertia.x, 1 / body.mass.inverseInertia.y, 1 / body.mass.inverseInertia.z};
    return body.orientation.rotate(inertia.scaled(body.orientation.rotateInverse(w)));
}

// Тела в движении внутри шага. origin и facing — позы начала кадра: от них считаются сдвиг и поворот,
// по которым контакт знает нынешний зазор. Позы и скорости перед последним сдвигом запоминаются:
// к ним шаг возвращается, если сдвиг приходится повторить (MissedContacts).
// job(i) для count независимых заданий: каждое пишет только в своё место, поэтому ответ от числа потоков
// не зависит. Раздаются пулу кусками — поток читает память подряд; малое число дешевле сделать на месте.
template <class Job> void eachOf(size_t count, const Job& job) {
    constexpr size_t kChunk = 32, kParallelChunks = 8;
    const size_t chunks = (count + kChunk - 1) / kChunk;
    if (chunks < kParallelChunks) {
        for (size_t i = 0; i < count; ++i) job(i);
        return;
    }
    WorkerPool::shared().run(chunks, [&](size_t k) {
        for (size_t i = k * kChunk; i < std::min(count, (k + 1) * kChunk); ++i) job(i);
    });
}

struct Moving {
    struct Pose {
        Vector3 position, angularMomentum;
        Quaternion orientation;
    };
    std::vector<Body>& bodies;
    StepArray<SolverBody>& states;
    StepArray<Pose> origin, before;
    StepArray<SolverBody> statesBefore;

    Moving(std::vector<Body>& all, StepArray<SolverBody>& solved) : bodies(all), states(solved) { origin = poses(); }
    StepArray<Pose> poses() const {
        StepArray<Pose> now(bodies.size());
        for (size_t i = 0; i < bodies.size(); ++i) now[i] = {bodies[i].position, bodies[i].angularMomentum, bodies[i].orientation};
        return now;
    }
    // Тела сдвигаются с найденными скоростями. Момент импульса при повороте сохраняется, а угловая
    // скорость пересчитывается по новой ориентации: это и есть гироскопический член.
    void advance(Real h) {
        before = poses(), statesBefore = states;
        eachOf(bodies.size(), [&](size_t i) {
            SolverBody& s = states[i];
            if (s.inverseMass == 0) return;
            Body& body = bodies[i];
            const Vector3 momentum = momentumOf(body, s.w);
            body.position += h * s.v;
            body.angularMomentum = momentum;
            // У тела с неравными моментами инерции угловая скорость меняется вместе с ориентацией,
            // поэтому поворот идёт со скоростью середины подшага. Со скоростью его начала вытянутое
            // тело само раскручивалось вокруг длинной оси: на 2 % за кадр.
            const Vector3 inverse = body.mass.inverseInertia;
            const bool round = inverse.x == inverse.y && inverse.y == inverse.z;
            const Vector3 turn = round ? h * s.w : h * angularVelocityAt(body, (Quaternion::fromRotationVector((h / 2) * s.w) * body.orientation).unit());
            body.orientation = (Quaternion::fromRotationVector(turn) * body.orientation).unit();
            s.moved = body.position - origin[i].position, s.turned = body.orientation * origin[i].orientation.conjugate();
            s.w = body.angularVelocity();
            s.inverseInertia = inverseInertiaInWorld(body);
        });
    }
    // Скорости решателя — в импульсы тел: по ним считают досягаемость PairReach и PairFinder.
    void sync() {
        for (size_t i = 0; i < bodies.size(); ++i)
            if (states[i].inverseMass != 0) bodies[i].linearMomentum = (1 / states[i].inverseMass) * states[i].v, bodies[i].angularMomentum = momentumOf(bodies[i], states[i].w);
    }
    void undo() {
        states = statesBefore;
        for (size_t i = 0; i < bodies.size(); ++i) bodies[i].position = before[i].position, bodies[i].angularMomentum = before[i].angularMomentum, bodies[i].orientation = before[i].orientation;
    }
};

// Связи сочленений по нынешним позам тел: запертые свободы — одной системой (DirectJoints),
// остальные — мягкими связями по одной. Строятся заново после каждого сдвига тел: проход без
// поправки идёт уже по новому положению, и следующий подшаг начинает с тех же связей. Половина
// импульса связи приходится на старое направление, половина — на новое; если оба прохода вести по
// старому, скорость каждый подшаг срезается на косинус поворота, и маятник теряет 2,5 % энергии в секунду.
struct JointRows {
    StepArray<SoftJoint> links;
    DirectJoints locked;

    void build(const JointSet& joints, const std::vector<Body>& bodies, const StepArray<SolverBody>& states, std::vector<std::vector<Real>>& memory, bool direct) {
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
    void warmStart(StepArray<SolverBody>& states) const {
        locked.warmStart(states);
        for (const SoftJoint& link : links) link.warmStart(states);
    }
    void solve(StepArray<SolverBody>& states, bool withBias, Real correction, Real h, const Softness& soft, Real pushSpeed) {
        locked.solve(states, withBias ? correction : 0, h);
        for (SoftJoint& link : links) link.solve(states, withBias, h, soft, pushSpeed);
    }
};

// Начало подшага: сопротивление среды, v ← v / (1 + h c) (Body::linearDamping), и тяжесть.
void applyForces(StepArray<SolverBody>& states, const std::vector<Body>& bodies, Vector3 gravity, Real h) {
    for (size_t i = 0; i < states.size(); ++i) {
        if (states[i].inverseMass <= 0) continue;
        states[i].v = (1 / (1 + h * bodies[i].linearDamping)) * states[i].v + h * gravity;
        states[i].w = (1 / (1 + h * bodies[i].angularDamping)) * states[i].w;
    }
}

constexpr int kRechecks = 3; // сколько раз подшаг повторяет сдвиг, найдя пропущенные контакты

// Контакты шага и то, что пополняет их по ходу: пропущенные точки пар списка (MissedContacts) и
// пары, которых в списке не было (PairReach).
struct LiveContacts {
    StepArray<SoftContact>& contacts;
    IslandNormals& normals;
    MissedContacts& missed;
    PairReach& reach;
    const ContactMaker& maker;
    const JointSet& joints;
    Real pushSpeed, bounceSpeed, boundary;
    const StageObserver& observer;
    bool bounces; // есть ли тело с отскоком: без него проход отскока по контактам не нужен

    // true — какое-то тело выходит за досягаемость списка пар, и нашлись новые пары с точками контакта.
    bool discover(Moving& moving, Real h, Real remaining) {
        moving.sync();
        if (!reach.exceeded(moving.bodies, h)) return false;
        bool added = false;
        for (const ContactPair& pair : reach.discover(moving.bodies, joints, remaining, boundary)) {
            missed.adopt(pair, moving.bodies, pair.manifold.count > 0);
            if (pair.manifold.count > 0) contacts.push_back(maker.make(pair, true)), added = true;
        }
        return added;
    }
    // Нормали с поправкой положения и отскок точек, принявших удар.
    void impact(Moving& moving, Real h) {
        normals.solve(contacts, moving.bodies, moving.states, true, h, pushSpeed);
        if (!bounces) return;
        for (SoftContact& contact : contacts) contact.bounce(moving.states, bounceSpeed);
    }
    // Нормали с поправкой положения и сдвиг тел на h; remaining — время до конца обновления. Сдвиг
    // пробный: если после него нашлись пропущенные контакты, тела возвращаются и нормали решаются
    // снова, уже с ними.
    void solveAndMove(Moving& moving, Real h, Real remaining) {
        const MissedContacts::Maker untracked = [&](const ContactPair& pair) { return maker.make(pair, false); };
        for (int round = 0;; ++round) {
            impact(moving, h), notify(observer, StepStage::Normals);
            for (int k = 0; k < kRechecks && discover(moving, h, remaining); ++k) impact(moving, h);
            notify(observer, StepStage::Recheck);
            missed.forget(), moving.advance(h);
            StepArray<Real> depth(contacts.size()); // глубина до пересчёта: её записывает общий проход ниже
            eachOf(contacts.size(), [&](size_t c) { SoftContact::prefetchAhead(contacts, moving.states, c), depth[c] = deepestOf(contacts[c]), contacts[c].follow(moving.states); });
            for (size_t c = 0; c < contacts.size(); ++c) missed.note(c, depth[c]);
            notify(observer, StepStage::Motion);
            const size_t known = contacts.size();
            const bool found = round != kRechecks && missed.find(contacts, moving.bodies, moving.states, h, untracked);
            notify(observer, StepStage::Recheck);
            if (!found) break;
            moving.undo();
            eachOf(contacts.size(), [&](size_t c) { contacts[c].follow(moving.states); });
            for (size_t c = known; c < contacts.size(); ++c) contacts[c].matchTrial(moving.states, h);
        }
        reach.spend();
    }
};

// Какие тела движутся: скорость точки тела больше speed. Выталкивание вдавленных тел разгоняет их и в
// покое: тогда второе решение нормалей нужно, чтобы снять эту скорость (без него тела, вдавленные на
// 10 см, разлетались на 1,7 м/с).
StepArray<char> stirred(const std::vector<Body>& bodies, const StepArray<SolverBody>& states, Real speed) {
    StepArray<char> moving(bodies.size(), 0);
    for (size_t i = 0; i < bodies.size(); ++i) moving[i] = states[i].inverseMass != 0 && states[i].v.length() + states[i].w.length() * bodies[i].boundingRadius() > speed;
    return moving;
}

// Конец шага: скорости решателя — в импульсы тел, накопленные импульсы контактов — в память.
void finish(std::vector<Body>& bodies, const StepArray<SolverBody>& states, const StepArray<SoftContact>& contacts, ContactForces& memory, Real h) {
    for (size_t i = 0; i < bodies.size(); ++i)
        if (states[i].inverseMass != 0) bodies[i].linearMomentum = (1 / states[i].inverseMass) * states[i].v, bodies[i].angularMomentum = momentumOf(bodies[i], states[i].w);
    memory.clear();
    for (const SoftContact& contact : contacts) contact.remember(memory, h);
}

} // namespace

void SoftStep::step(std::vector<Body>& bodies, const std::vector<ContactPair>& pairs, const JointSet& joints, Vector3 gravity, Real dt, ContactForces& contactMemory,
                    std::vector<std::vector<Real>>& jointMemory) const {
    const Real h = dt / substeps;
    const Real hertz = std::min(contactHertz, Real(0.125) / h);
    const Softness soft = Softness::make(hertz, contactDamping, h), staticSoft = Softness::make(2 * hertz, contactDamping / 2, h);
    const Softness jointSoft = Softness::make(std::min(jointHertz, Real(0.25) / h), jointDamping, h);

    for (const Joint& joint : joints) joint.applyDrive(bodies, dt);
    StepArray<SolverBody> states = solverBodiesOf(bodies);
    const ContactMaker maker{bodies, states, soft, staticSoft, h, fourContactPoints};
    StepArray<size_t> owner;
    StepArray<SoftContact> contacts = maker.all(pairs, contactMemory, owner);
    MissedContacts missed(pairs, bodies, owner, recheckDepth, edgeSlop);
    PairReach reach(bodies, pairs, gravity, dt, recheckDepth / 2);
    Moving moving(bodies, states);
    jointMemory.resize(joints.size());
    JointRows rows;
    rows.build(joints, bodies, states, jointMemory, directJoints);
    IslandNormals normals(directContactRows, soft.biasRate);
    LiveContacts live{contacts, normals, missed, reach, maker, joints, pushSpeed, bounceSpeed, edgeSlop, observer,
                      std::any_of(bodies.begin(), bodies.end(), [](const Body& body) { return body.restitution != 0; })};
    notify(observer, StepStage::Contacts);
    for (int substep = 0; substep < substeps; ++substep) {
        applyForces(states, bodies, gravity, h);
        // Тёплый старт: накопленные импульсы прикладываются к новым скоростям.
        rows.warmStart(states);
        for (size_t c = 0; c < contacts.size(); ++c) SoftContact::prefetchAhead(contacts, states, c), contacts[c].warmStart(states);
        notify(observer, StepStage::WarmStart);
        rows.solve(states, true, jointCorrection, h, jointSoft, pushSpeed), notify(observer, StepStage::Joints);
        live.solveAndMove(moving, h, (substeps - substep) * h);
        rows.build(joints, bodies, states, jointMemory, directJoints);
        rows.solve(states, false, jointCorrection, h, jointSoft, pushSpeed), notify(observer, StepStage::Joints);
        const StepArray<char> moving = stirred(bodies, states, restSpeed);
        if (restSpeed == 0 || std::find(moving.begin(), moving.end(), 1) != moving.end()) normals.solve(contacts, bodies, states, false, h, pushSpeed, restSpeed > 0 ? &moving : nullptr);
        notify(observer, StepStage::Normals);
        for (size_t c = 0; c < contacts.size(); ++c) SoftContact::prefetchAhead(contacts, states, c), contacts[c].solveFriction(states, rollingResistance);
        notify(observer, StepStage::Friction);
    }
    finish(bodies, states, contacts, contactMemory, h), notify(observer, StepStage::Memory);
}

} // namespace kp
