// Пробный подшаг для почти неподвижных тел. p в Н·с, L в Н·м·с, оси мира.
constexpr Real kSupportSpeed = Real(1e-8);
struct TrialMomentum {
    Vector3 p, L, v, w;
};
struct PatchFriction { Vector3 tangent; Real twist = 0; };

bool slowEnough(Vector3 v, Vector3 w, Real radius, Real h) {
    const Real spin = w.length(), speed = v.length() + radius * spin;
    return speed <= kSupportSpeed && spin <= kSupportSpeed && speed * h <= Real(1e-10);
}

bool addGuideReactions(const JointSet& joints, const std::vector<Body>& bodies, const std::vector<std::vector<Real>>& previous,
                       StepArray<TrialMomentum>& trial, Real h) {
    if (joints.size() != previous.size()) return false;
    std::vector<Bond> rows;
    if (!joints.empty()) rows.reserve(6);
    for (size_t j = 0; j < joints.size(); ++j) {
        joints[j].writeBonds(bodies, bodies, rows);
        if (previous[j].size() != rows.size()) return false;
        for (size_t k = 0; k < rows.size(); ++k) {
            const Bond& row = rows[k];
            if (row.soft || row.driven || row.oneSided || !(abs(row.error) <= h * kSupportSpeed)) return false;
            const Real impulse = previous[j][k];
            const Vector3 along = impulse * row.linear;
            auto &a = trial[size_t(row.a)], &b = trial[size_t(row.b)];
            a.p -= along;
            b.p += along;
            a.L -= impulse * row.angularA;
            b.L += impulse * row.angularB;
        }
    }
    return true;
}

bool addPatchReaction(const ContactPair& pair, const std::vector<Body>& bodies, Span<ContactForces::Entry> previous,
                      StepArray<TrialMomentum>& trial, Real h, PatchFriction& carried) {
    const auto& m = pair.manifold;
    const Body &a = bodies[size_t(pair.a)], &b = bodies[size_t(pair.b)];
    if (m.count < 2 || m.count > 4 || previous.size() != size_t(m.count) || m.roundA != 0 || m.roundB != 0 ||
        a.restitution != 0 || b.restitution != 0 || !a.shape || !b.shape || !a.shape->hull() || !b.shape->hull() || a.parts || b.parts) return false;
    std::array<Vector3, 4> arms{};
    std::array<Real, 4> weight{};
    Vector3 centroid{}, normalArm{}, tangent{};
    Real load = 0, twist = 0;
    unsigned used = 0;
    for (int k = 0; k < m.count; ++k) {
        const auto& point = m.points[size_t(k)];
        if (!(abs(point.separation) <= h * kSupportSpeed)) return false;
        size_t j = 0;
        while (j < previous.size() && previous[j].first.id != point.id) ++j;
        if (j == previous.size() || (used & (1u << j))) return false;
        used |= 1u << j;
        const ContactForce& f = previous[j].second;
        if (!(f.normal >= 0)) return false;
        // Сначала вычитаем центр масс, сохраняя точность вдали от начала координат.
        arms[size_t(k)] = Real(0.5) * ((point.onA - a.position) + (point.onB - a.position));
        weight[size_t(k)] = f.normal;
        centroid += arms[size_t(k)];
        normalArm += f.normal * arms[size_t(k)];
        tangent += f.tangent;
        twist += f.twist;
        load += f.normal;
    }
    if (!(load > 0)) return false;
    centroid = (Real(1) / m.count) * centroid;
    Real lever = 0;
    for (int k = 0; k < m.count; ++k) lever += weight[size_t(k)] * (arms[size_t(k)] - centroid).length();
    const Real mu = sqrt(a.friction * b.friction);
    tangent -= tangent.dot(m.normal) * m.normal;
    const Real length = tangent.length(), limit = mu * load;
    if (!(limit >= 0) || !(length >= 0)) return false;
    if (length > limit) tangent = (limit / length) * tangent;
    twist = std::clamp(twist, -mu * lever, mu * lever);
    carried = {(Real(1) / m.count) * tangent, twist / m.count};
    const Vector3 impulse = h * (load * m.normal + tangent);
    const Vector3 torqueA = h * (normalArm.cross(m.normal) + centroid.cross(tangent) + twist * m.normal);
    auto &sa = trial[size_t(pair.a)], &sb = trial[size_t(pair.b)];
    sa.p -= impulse;
    sb.p += impulse;
    sa.L -= torqueA;
    sb.L += torqueA + (a.position - b.position).cross(impulse);
    return true;
}

bool tryMomentumStep(const SoftStep& step, std::vector<Body>& bodies, const std::vector<ContactPair>& pairs, const JointSet& joints,
                     Vector3 gravity, Real h, ContactForces& forces, const std::vector<std::vector<Real>>& jointMemory) {
    if (!(h > 0) || step.substeps != 1 || !step.directJoints || !step.coupledFriction ||
        step.rollingResistance != 0 || pairs.empty() || forces.begin() == forces.end() || step.directContactRows <= 0) return false;
    size_t points = 0;
    for (const auto& pair : pairs) points += size_t(pair.manifold.count);
    if (points > size_t(step.directContactRows) || points != size_t(forces.end() - forces.begin())) return false;
    StepArray<TrialMomentum> trial(bodies.size());
    for (size_t i = 0; i < bodies.size(); ++i) {
        const Body& body = bodies[i];
        if (isDriven(body)) return false;
        if (body.isStatic()) continue;
        if (!slowEnough(body.linearVelocity(), body.angularVelocity(), body.shape ? body.boundingRadius() : Real(0), h)) return false;
        trial[i].p = (1 / (1 + h * body.linearDamping)) * body.linearMomentum + h * (body.force + (1 / body.mass.inverseMass) * gravity);
        trial[i].L = (1 / (1 + h * body.angularDamping)) * body.angularMomentum + h * body.torque;
    }
    if (!addGuideReactions(joints, bodies, jointMemory, trial, h)) return false;
    StepArray<PatchFriction> friction(pairs.size());
    const auto* hint = forces.begin();
    for (size_t i = 0; i < pairs.size(); ++i) {
        const auto& pair = pairs[i];
        const auto previous = forces.of({pair.a, pair.b, 0, pair.partA, pair.partB}, hint);
        if (!addPatchReaction(pair, bodies, previous, trial, h, friction[i])) return false;
        hint = previous.end();
    }
    for (size_t i = 0; i < bodies.size(); ++i) {
        const Body& body = bodies[i];
        if (body.isStatic()) continue;
        auto& t = trial[i];
        t.v = body.mass.inverseMass * t.p;
        t.w = body.orientation.rotate(body.mass.inverseInertia.scaled(body.orientation.rotateInverse(t.L)));
        if (!slowEnough(t.v, t.w, body.shape ? body.boundingRadius() : Real(0), h)) return false;
    }
    for (StepStage stage : {StepStage::Contacts,StepStage::WarmStart,StepStage::Joints,StepStage::Normals,StepStage::Recheck})
        notify(step.observer, stage);
    for (size_t i = 0; i < bodies.size(); ++i) {
        Body& body = bodies[i];
        if (body.isStatic()) continue;
        const auto& t = trial[i];
        body.linearMomentum = t.p;
        body.angularMomentum = t.L;
        body.position += h * t.v;
        const auto inverse = body.mass.inverseInertia;
        const bool round = inverse.x == inverse.y && inverse.y == inverse.z;
        const Vector3 turn = round ? h * t.w : h * angularVelocityAt(body, (Quaternion::fromRotationVector((h / 2) * t.w) * body.orientation).unit());
        body.orientation = (Quaternion::fromRotationVector(turn) * body.orientation).unit();
    }
    for (size_t i = 0; i < pairs.size(); ++i)
        forces.reviseFriction({pairs[i].a, pairs[i].b, 0, pairs[i].partA, pairs[i].partB}, friction[i].tangent, friction[i].twist);
    for (StepStage stage : {StepStage::Motion,StepStage::Recheck,StepStage::Joints,StepStage::Normals,StepStage::Friction,StepStage::Memory})
        notify(step.observer, stage);
    return true;
}
