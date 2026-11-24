#include <collision/narrowphase/RoundVsRoundAlgorithm.h>

#include <collision/narrowphase/ContactCandidate.h>
#include <collision/narrowphase/ContactPatch.h>
#include <collision/shapes/Tube.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace kp {
namespace {

// Точка оси трубки upper над трубкой lower: точки на двух поверхностях и зазор — расстояние между
// осями без радиусов. Первая точка ответа — на lower.
ContactPoint above(const Tube& lower, const Tube& upper, Vector3 onAxis, std::uint32_t id) {
    const Vector3 nearest = lower.centre + lower.axis.dot(onAxis - lower.centre) * lower.axis;
    const Real distance = (onAxis - nearest).length();
    const Vector3 away = (1 / distance) * (onAxis - nearest);
    return {nearest + lower.radius * away, onAxis - upper.radius * away, distance - lower.radius - upper.radius, id};
}

// Часть оси трубки tube, лежащая над трубкой lower, — общий участок вдоль её оси. false, если
// такой части нет.
bool overlapOf(const Tube& tube, const Tube& lower, Vector3& from, Vector3& to) {
    const Vector3 start = tube.centre - tube.halfLength * tube.axis, along = (2 * tube.halfLength) * tube.axis;
    const Real s0 = lower.axis.dot(start - lower.centre), rate = lower.axis.dot(along);
    if (abs(rate) < kOnBoundary) return false; // оси скрещиваются под прямым углом: общего участка нет
    const Real t0 = (-lower.halfLength - s0) / rate, t1 = (lower.halfLength - s0) / rate;
    const Real enter = std::max(Real(0), std::min(t0, t1)), leave = std::min(Real(1), std::max(t0, t1));
    if (enter > leave) return false;
    from = start + enter * along, to = start + leave * along;
    return true;
}

} // namespace

ContactManifold RoundVsRoundAlgorithm::collide(const ConvexShape& a, const Transform& poseA, const ConvexShape& b, const Transform& poseB, const ClosestPoints& closest, Real margin,
                                               Real touching, Real) const {
    // Две круглые формы с перекрывшимися ядрами здесь не разрешаются.
    if (closest.coresOverlap || closest.distance > margin) return {};
    // У двух совпадающих точечных ядер нормаль выбрана при расчёте свидетелей.
    // Торец цилиндра в общем случае даёт площадку, в том числе при глубоком перекрытии.
    const bool points = closest.distance <= -a.radius - b.radius && a.hasPointCore() && b.hasPointCore();
    if (!points) {
        const ContactManifold patch = ContactPatch::between(a, poseA, b, poseB, closest, margin, touching);
        if (patch.count > 0) return patch;
    }
    ContactManifold single;
    single.normal = closest.normal;
    single.add({closest.onA, closest.onB, closest.distance, 0});
    single.roundA = a.rollingRadius(poseA, closest.normal);
    single.roundB = b.rollingRadius(poseB, -1 * closest.normal);
    if (points) return single;

    // Плоского элемента нет: контакт — ближайшие точки. Две трубки, лежащие вдоль друг друга,
    // касаются по отрезку, и к ближайшим точкам добавляются точки на концах их общего участка.
    // Сама точка касания остаётся всегда: у трубок, лежащих крест-накрест, концы общего участка
    // далеко от неё, и без неё у пары не было ни опоры, ни того, чем раздвигать тела, — малый
    // цилиндр на большом проваливался на допуск за каждый отрезок шага.
    const std::optional<Tube> ta = a.tubeTowards(poseA, closest.normal), tb = b.tubeTowards(poseB, -1 * closest.normal);
    Vector3 from, to;
    if (!ta || !tb || !overlapOf(*tb, *ta, from, to)) return single;
    const ContactPoint found[3] = {single.points[0], above(*ta, *tb, from, 1), above(*ta, *tb, to, 2)};
    // Если касаются оба конца общего участка, трубки лежат вдоль друг друга, и ближайшая точка —
    // случайная точка между концами: площадку задают концы.
    const bool along = found[1].separation <= touching && found[2].separation <= touching;
    ContactCandidates candidates;
    // Ни одна точка контакта не бывает ближе ближайших точек тел. Цилиндр, упёртый кромкой под
    // 45°, считался трубкой, и концы «общего участка» получали зазор −7 см; выталкивание по ним
    // подбрасывало тело вверх на каждом отрезке, и оно никогда не долетало до второго.
    for (std::uint32_t i = along ? 1 : 0; i < 3; ++i)
        if (i == 0 || (found[i].separation <= margin && found[i].separation >= closest.distance - kOnBoundary)) candidates.push_back({found[i].onA, found[i].separation, i});
    ContactManifold manifold;
    manifold.normal = closest.normal;
    manifold.roundA = single.roundA, manifold.roundB = single.roundB;
    for (const ContactCandidate& kept : ContactCandidate::ordered(candidates, touching)) manifold.add(found[kept.id]);
    return manifold;
}

} // namespace kp
