#include <collision/ManifoldMemory.h>

#include <algorithm>
#include <limits>

namespace kp {
namespace {

bool smallerIsA(const Body& a, const Body& b) { return a.shape->boundingRadius() <= b.shape->boundingRadius(); }

// Поза меньшего тела в осях большего: смещение любой точки меньшего тела относительно большего
// ограничено смещением его центра и поворотом на его же радиусе.
Transform apartOf(const Body& a, const Body& b) { return smallerIsA(a, b) ? b.pose().relative(a.pose()) : a.pose().relative(b.pose()); }

} // namespace

std::optional<ManifoldMemory::Entry> ManifoldMemory::entryOf(const ContactPair& pair, const Body& a, const Body& b) {
    const ContactManifold& found = pair.manifold;
    if (!a.shape->hull() || !b.shape->hull() || found.count < 2 || found.count > kPoints || found.roundA != 0 || found.roundB != 0) return std::nullopt;
    const Transform poseA = a.pose(), poseB = b.pose();
    const Transform &face = found.faceOnB ? poseB : poseA, &lying = found.faceOnB ? poseA : poseB;
    const auto onFace = [&](int k) { return found.faceOnB ? found.points[size_t(k)].onB : found.points[size_t(k)].onA; };
    const auto onLying = [&](int k) { return found.faceOnB ? found.points[size_t(k)].onA : found.points[size_t(k)].onB; };
    Entry entry{pair.a, pair.b, pair.partA, pair.partB, a.shape.get(), b.shape.get(), apartOf(a, b), pair.reach, found.faceOnB, face.localDirection(found.normal), face.localPoint(onFace(0)), found.count, {}};
    for (int k = 0; k < found.count; ++k) entry.points[k] = {lying.localPoint(onLying(k)), found.points[size_t(k)].id};
    return entry;
}

// a и b — части, как их отдаёт Body::part: у простого тела оно само. key — {тело a, тело b, часть a, часть b}.
const ManifoldMemory::Entry* ManifoldMemory::valid(const std::array<int, 4>& key, const Body& a, const Body& b, Real reach) const {
    const auto before = [](const Entry& entry, const std::array<int, 4>& wanted) {
        if (entry.a != wanted[0]) return entry.a < wanted[0];
        if (entry.b != wanted[1]) return entry.b < wanted[1];
        return entry.partA != wanted[2] ? entry.partA < wanted[2] : entry.partB < wanted[3];
    };
    const auto known = std::lower_bound(entries_.begin(), entries_.end(), key, before);
    if (known == entries_.end() || known->a != key[0] || known->b != key[1] || known->partA != key[2] || known->partB != key[3]) return nullptr;
    if (known->shapeA != a.shape.get() || known->shapeB != b.shape.get()) return nullptr;
    const Transform now = apartOf(a, b);
    // 2 sin(θ/2) — удвоенная длина векторной части кватерниона относительного поворота.
    const Quaternion turn = now.orientation * known->apart.orientation.conjugate();
    const Real chord = 2 * Vector3{turn.x, turn.y, turn.z}.length();
    const Real drift = (now.position - known->apart.position).length() + chord * std::min(a.shape->boundingRadius(), b.shape->boundingRadius());
    return drift + std::max(Real(0), reach - known->reach) <= tolerance ? &*known : nullptr;
}

// Точка лежащего тела идёт с ним; её зазор — расстояние до плоскости опорной грани вдоль нормали
// площадки (от A к B), а точка на грани — её проекция на эту плоскость (HullVsHullAlgorithm).
ContactPair ManifoldMemory::carried(const Entry& entry, const Body& a, const Body& b, Real reach) {
    ContactPair pair;
    carryInto(entry, a, b, reach, pair);
    return pair;
}

void ManifoldMemory::carryInto(const Entry& entry, const Body& a, const Body& b, Real reach, ContactPair& pair) {
    const Transform poseA = a.pose(), poseB = b.pose();
    const Transform &face = entry.faceOnB ? poseB : poseA, &lying = entry.faceOnB ? poseA : poseB;
    pair.manifold.points.clear();
    pair.manifold.count = 0;
    pair.manifold.roundA = pair.manifold.roundB = 0;
    pair.reach = reach, pair.manifold.faceOnB = entry.faceOnB;
    const Vector3 n = pair.manifold.normal = face.direction(entry.normal), onFace = face.point(entry.onFace);
    Real least = std::numeric_limits<Real>::infinity();
    for (int k = 0; k < entry.count; ++k) {
        const Vector3 at = lying.point(entry.points[k].at);
        const Real separation = entry.faceOnB ? n.dot(onFace - at) : n.dot(at - onFace);
        pair.manifold.add(entry.faceOnB ? ContactPoint{at, at + separation * n, separation, entry.points[k].id} : ContactPoint{at - separation * n, at, separation, entry.points[k].id});
        least = std::min(least, separation);
    }
    pair.distance = std::max(Real(0), least);
}

// Записи, оставшиеся на своих местах, не копируются: в покое состав памяти не меняется вовсе. Иначе
// новый состав собирается в стороне — указатели смотрят в нынешний — и занимает его место.
void ManifoldMemory::replace(const StepArray<const Entry*>& entries) {
    size_t same = 0;
    while (same < entries.size() && same < entries_.size() && entries[same] == &entries_[same]) ++same;
    if (same == entries.size() && same == entries_.size()) return;
    next_.clear(), next_.reserve(entries.size());
    for (const Entry* entry : entries) next_.push_back(*entry);
    entries_.swap(next_);
}

} // namespace kp
