#include <collision/narrowphase/ContactCandidate.h>

#include <algorithm>
#include <limits>

namespace kp {
namespace {

// Точки, чьи глубины отличаются меньше чем на это, считаются одинаково глубокими. У плоского
// контакта все глубины равны с точностью до округления, и без запаса «самую глубокую» выбирал
// бы шум — каждый шаг другую.
constexpr Real kEqualDepth = Real(1e-7); // м

} // namespace

// Точки в порядке, в каком их получает решатель: сначала опора — точки, которые уже касаются
// (зазор не больше touching или такой же, как у самой глубокой), в порядке построения; за ними
// остальные по возрастанию зазора — они коснутся следующими, и поиск времени удара обязан о них
// знать. Ни одна точка не отбрасывается. Порядок зависит только от самих чисел.
ContactCandidates ContactCandidate::ordered(const ContactCandidates& points, Real touching) {
    Real deepest = std::numeric_limits<Real>::infinity();
    for (const ContactCandidate& p : points) deepest = std::min(deepest, p.separation);
    const Real supportLevel = std::max(deepest + kEqualDepth, touching);
    ContactCandidates support, approaching;
    for (const ContactCandidate& p : points) (p.separation <= supportLevel ? support : approaching).push_back(p);
    // Устойчивая сортировка вставками: порядок равных сохраняется, как у std::stable_sort, а к куче она
    // не обращается.
    for (size_t i = 1; i < approaching.size(); ++i)
        for (size_t k = i; k > 0 && approaching[k].separation < approaching[k - 1].separation; --k) std::swap(approaching[k], approaching[k - 1]);
    for (const ContactCandidate& p : approaching) support.push_back(p);
    return support;
}

} // namespace kp
