#include <math/StepMemory.h>
#include <dynamics/event_driven/OverlapSeparation.h>

#include <dynamics/event_driven/ContactSolver.h>
#include <dynamics/Integrator.h>

#include <algorithm>

namespace kp {

// Точки контакта меряют зазор от плоскости грани, а у её края тела входят друг в друга и
// мимо этой плоскости: ребро оболочки вдавлено в бок цилиндра на 7,7 мкм, а точка на грани
// показывает 4,8. По таким точкам раздвигание не срабатывало, хотя поиск начала перекрытия,
// который смотрит на истинную глубину, уже не пускал тела дальше, — шаг вставал. Поэтому,
// если истинная глубина больше той, что показывают точки, пара раздвигается по истинной
// нормали на истинную глубину, в самой глубокой своей точке.
bool OverlapSeparation::deepen(std::vector<ContactPair>& pushed) const {
    bool overlapping = false;
    for (ContactPair& pair : pushed) {
        int deepest = -1;
        for (int i = 0; i < pair.manifold.count; ++i)
            if (deepest < 0 || pair.manifold.points[i].separation < pair.manifold.points[size_t(deepest)].separation) deepest = i;
        if (deepest >= 0 && pair.inContact()) {
            const Separation separation = separationOf(bodies_[size_t(pair.a)].part(pair.partA), bodies_[size_t(pair.b)].part(pair.partB));
            if (separation.distance < pair.manifold.points[size_t(deepest)].separation - kOverlap / 4) {
                ContactPoint point = pair.manifold.points[size_t(deepest)];
                point.separation = separation.distance;
                pair.manifold.points.clear();
                pair.manifold.add(point);
                pair.manifold.normal = separation.normal;
            }
        }
        for (int i = 0; i < pair.manifold.count; ++i) overlapping = overlapping || pair.manifold.points[size_t(i)].separation < -kOverlap / 2;
    }
    return overlapping;
}

bool OverlapSeparation::separate(const std::vector<ContactPair>& touching) {
    std::vector<ContactPair> pushed = touching;
    // Разошедшееся сочленение возвращается на место тем же способом: сдвигом поз без скорости.
    const bool overlapping = deepen(pushed) || joints_.largestError(bodies_) > kOverlap / 2;
    if (!overlapping) return false;

    std::vector<Body> shadows = bodies_;
    for (Body& shadow : shadows) shadow.linearMomentum = shadow.angularMomentum = {};
    ContactSolver(shadows, pushed, up_, 0, nullptr, &joints_).solveOverlap();
    StepArray<Vector3> shifts(bodies_.size());
    for (size_t i = 0; i < bodies_.size(); ++i) {
        integratePosition(shadows[i], 1);
        shifts[i] = shadows[i].position - bodies_[i].position;
        bodies_[i].position = shadows[i].position;
        bodies_[i].orientation = shadows[i].orientation;
    }
    keepMomenta(pushed, shifts);
    return true;
}

StepArray<int> OverlapSeparation::freeGroups(const std::vector<ContactPair>& pairs) const {
    StepArray<int> group(bodies_.size());
    for (size_t i = 0; i < group.size(); ++i) group[i] = int(i);
    const auto root = [&](int i) {
        while (group[size_t(i)] != i) i = group[size_t(i)] = group[size_t(group[size_t(i)])];
        return i;
    };
    const auto link = [&](int a, int b) { group[size_t(std::max(root(a), root(b)))] = std::min(root(a), root(b)); };
    for (const ContactPair& pair : pairs)
        if (pair.manifold.count > 0) link(pair.a, pair.b);
    for (const Joint& joint : joints_) link(joint.a, joint.b);
    StepArray<char> grounded(bodies_.size(), 0);
    for (size_t i = 0; i < bodies_.size(); ++i)
        if (bodies_[i].isStatic()) grounded[size_t(root(int(i)))] = 1;
    StepArray<int> roots(bodies_.size());
    for (size_t i = 0; i < roots.size(); ++i) roots[i] = grounded[size_t(root(int(i)))] ? -1 : root(int(i));
    return roots;
}

void OverlapSeparation::keepMomenta(const std::vector<ContactPair>& pairs, const StepArray<Vector3>& shifts) {
    const StepArray<int> group = freeGroups(pairs);
    const auto root = [&](int i) { return group[size_t(i)]; };
    for (int g = 0; g < int(bodies_.size()); ++g) {
        if (root(g) != g) continue;
        Real mass = 0;
        Vector3 centre, lost;
        for (size_t i = 0; i < bodies_.size(); ++i)
            if (root(int(i)) == g) mass += 1 / bodies_[i].mass.inverseMass, centre += (1 / bodies_[i].mass.inverseMass) * bodies_[i].position, lost += shifts[i].cross(bodies_[i].linearMomentum);
        if (lost.dot(lost) == 0) continue;
        centre = (1 / mass) * centre;
        PairFriction::Response inertia{};
        const auto momentsOf = [](const Body& body) { return Vector3{1 / body.mass.inverseInertia.x, 1 / body.mass.inverseInertia.y, 1 / body.mass.inverseInertia.z}; };
        const auto spun = [&](const Body& body, Vector3 w) { return body.orientation.rotate(momentsOf(body).scaled(body.orientation.rotateInverse(w))); };
        const Vector3 axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        for (size_t i = 0; i < bodies_.size(); ++i) {
            if (root(int(i)) != g) continue;
            const Vector3 d = bodies_[i].position - centre;
            for (int c = 0; c < 3; ++c) {
                const Vector3 column = (1 / bodies_[i].mass.inverseMass) * d.cross(axes[c].cross(d)) + spun(bodies_[i], axes[c]);
                inertia[0][size_t(c)] += column.x, inertia[1][size_t(c)] += column.y, inertia[2][size_t(c)] += column.z;
            }
        }
        const PairFriction::Response inverse = PairFriction::inverted(inertia);
        const Vector3 turn{-(inverse[0][0] * lost.x + inverse[0][1] * lost.y + inverse[0][2] * lost.z), -(inverse[1][0] * lost.x + inverse[1][1] * lost.y + inverse[1][2] * lost.z),
                           -(inverse[2][0] * lost.x + inverse[2][1] * lost.y + inverse[2][2] * lost.z)};
        for (size_t i = 0; i < bodies_.size(); ++i) {
            if (root(int(i)) != g) continue;
            bodies_[i].linearMomentum += (1 / bodies_[i].mass.inverseMass) * turn.cross(bodies_[i].position - centre);
            bodies_[i].angularMomentum += spun(bodies_[i], turn);
        }
    }
}

} // namespace kp
