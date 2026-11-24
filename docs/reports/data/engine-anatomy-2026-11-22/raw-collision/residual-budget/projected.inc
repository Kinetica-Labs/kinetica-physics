// Опыт для одного жёсткого подшага. Все силы прикладываются к пробным скоростям.
// Допуск на скорость точки (м/с) ограничивает путь за подшаг величиной h * kBalanceSpeed.
constexpr Real kBalanceSpeed = Real(1e-8);

bool slowBodies(const std::vector<Body>& bodies, const StepArray<SolverBody>& states, Real h) {
    for (size_t i = 0; i < bodies.size(); ++i) {
        const auto& body = bodies[i];
        const auto& s = states[i];
        if (isDriven(body)) return false;
        if (body.isStatic()) continue;
        const Real radius = body.shape ? body.boundingRadius() : Real(0);
        const Real speed = s.v.length() + radius * s.w.length();
        if (!(speed <= kBalanceSpeed && h * speed <= Real(1e-10) && s.w.length() <= kBalanceSpeed)) return false;
    }
    return true;
}

// p = h F. Центр трения у почти плоской площадки — среднее её точек.
struct SupportFriction {
    Vector3 tangent;
    Real twist;
};
bool carryReaction(const ContactPair& pair, const std::vector<Body>& bodies, StepArray<SolverBody>& trial,
                   const ContactForces& forces, Real h, SupportFriction& next) {
    const auto& m = pair.manifold;
    const Body &a = bodies[size_t(pair.a)], &b = bodies[size_t(pair.b)];
    if (m.count < 2 || m.count > 4 || m.roundA != 0 || m.roundB != 0 || a.restitution != 0 || b.restitution != 0 ||
        !a.shape || !b.shape || !a.shape->hull() || !b.shape->hull() || a.parts || b.parts) return false;
    std::array<Real, 4> normal{};
    std::array<Vector3, 4> at{};
    Real load = 0, moment = 0;
    Vector3 centre{}, tangent{};
    auto &sa = trial[size_t(pair.a)], &sb = trial[size_t(pair.b)];
    for (int k = 0; k < m.count; ++k) {
        const auto& point = m.points[size_t(k)];
        // Близкий контакт обязан уже лежать на поверхности. Иначе нужна позиционная поправка.
        if (!(abs(point.separation) <= h * kBalanceSpeed)) return false;
        const auto* entry = forces.find({pair.a, pair.b, point.id, pair.partA, pair.partB});
        if (entry == forces.end() || !(entry->second.normal >= 0)) return false;
        for (int j = 0; j < k; ++j) if (m.points[size_t(j)].id == point.id) return false;
        const auto& f = entry->second;
        at[size_t(k)] = Real(0.5) * (point.onA + point.onB);
        centre += at[size_t(k)];
        normal[size_t(k)] = f.normal;
        load += f.normal;
        moment += f.twist;
        tangent += f.tangent;
        SolverBody::push(sa, sb, at[size_t(k)] - a.position, at[size_t(k)] - b.position, (h * f.normal) * m.normal);
    }
    if (!(load > 0)) return false;
    centre = (Real(1) / m.count) * centre;
    Real leverLoad = 0;
    for (int k = 0; k < m.count; ++k) leverLoad += (at[size_t(k)] - centre).length() * normal[size_t(k)];
    const Real mu = sqrt(a.friction * b.friction);
    // Переносим силу в нынешнюю касательную плоскость и на диск Кулона.
    tangent -= tangent.dot(m.normal) * m.normal;
    const Real length = tangent.length(), limit = mu * load;
    if (!(limit >= 0) || !(length >= 0)) return false;
    if (length > limit) tangent = (limit / length) * tangent;
    moment = std::clamp(moment, -mu * leverLoad, mu * leverLoad);
    next = {Real(1) / m.count * tangent, moment / m.count};
    SolverBody::push(sa, sb, centre - a.position, centre - b.position, h * tangent);
    SolverBody::twist(sa, sb, (h * moment) * m.normal);
    return true;
}

bool carryGuides(const JointSet& joints, const std::vector<Body>& bodies, StepArray<SolverBody>& trial,
                 const std::vector<std::vector<Real>>& memory, Real h) {
    if (memory.size() != joints.size()) return false;
    std::vector<Bond> rows;
    for (size_t j = 0; j < joints.size(); ++j) {
        joints[j].writeBonds(bodies, bodies, rows);
        if (rows.size() != memory[j].size()) return false;
        for (size_t k = 0; k < rows.size(); ++k) {
            const auto& row = rows[k];
            if (row.soft || row.driven || row.oneSided || !(abs(row.error) <= h * kBalanceSpeed)) return false;
            const Real impulse = memory[j][k];
            auto &a = trial[size_t(row.a)], &b = trial[size_t(row.b)];
            if (a.inverseMass != 0) a.v -= (a.inverseMass * impulse) * row.linear, a.w -= a.spinFrom(impulse * row.angularA);
            if (b.inverseMass != 0) b.v += (b.inverseMass * impulse) * row.linear, b.w += b.spinFrom(impulse * row.angularB);
        }
    }
    return true;
}

bool tryBalancedStep(const SoftStep& step, std::vector<Body>& bodies, const std::vector<ContactPair>& pairs,
                     const JointSet& joints, Vector3 gravity, Real h, ContactForces& contactMemory,
                     const std::vector<std::vector<Real>>& jointMemory, const StepArray<SolverBody>& initial) {
    if (!(h > 0) || step.substeps != 1 || !step.directJoints || !step.coupledFriction ||
        step.rollingResistance != 0 || pairs.empty() || contactMemory.begin() == contactMemory.end() ||
        !slowBodies(bodies, initial, h)) return false;
    size_t points = 0;
    for (const auto& pair : pairs) points += size_t(pair.manifold.count);
    if (step.directContactRows < 0 || points > size_t(step.directContactRows) ||
        points != size_t(contactMemory.end() - contactMemory.begin())) return false;
    StepArray<SolverBody> trial = initial;
    applyForces(trial, bodies, gravity, h);
    if (!carryGuides(joints, bodies, trial, jointMemory, h)) return false;
    StepArray<SupportFriction> reactions(pairs.size());
    for (size_t i = 0; i < pairs.size(); ++i)
        if (!carryReaction(pairs[i], bodies, trial, contactMemory, h, reactions[i])) return false;
    if (!slowBodies(bodies, trial, h)) return false;
    for (StepStage stage : {StepStage::Contacts, StepStage::WarmStart, StepStage::Joints, StepStage::Normals, StepStage::Recheck})
        notify(step.observer, stage);
    // Поза меняется с рассчитанными скоростями. Для принятого пробного шага откат уже не нужен.
    for (size_t i = 0; i < bodies.size(); ++i) {
        Body& body = bodies[i];
        const auto& s = trial[i];
        if (s.inverseMass == 0) continue;
        body.linearMomentum = (1 / s.inverseMass) * s.v;
        body.angularMomentum = momentumOf(body, s.w);
        body.position += h * s.v;
        const auto inverse = body.mass.inverseInertia;
        const bool round = inverse.x == inverse.y && inverse.y == inverse.z;
        const Vector3 turn = round ? h * s.w : h * angularVelocityAt(body, (Quaternion::fromRotationVector((h / 2) * s.w) * body.orientation).unit());
        body.orientation = (Quaternion::fromRotationVector(turn) * body.orientation).unit();
    }
    notify(step.observer, StepStage::Motion);
    for (size_t i = 0; i < pairs.size(); ++i)
        contactMemory.reviseFriction({pairs[i].a, pairs[i].b, 0, pairs[i].partA, pairs[i].partB}, reactions[i].tangent, reactions[i].twist);
    for (StepStage stage : {StepStage::Recheck, StepStage::Joints, StepStage::Normals, StepStage::Friction, StepStage::Memory})
        notify(step.observer, stage);
    return true;
}
