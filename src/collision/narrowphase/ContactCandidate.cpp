#include <kp/collision/narrowphase/ContactCandidate.h>

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
std::vector<ContactCandidate> ContactCandidate::ordered(const std::vector<ContactCandidate>& points, Real touching) {
    Real deepest = std::numeric_limits<Real>::infinity();
    for (const ContactCandidate& p : points) deepest = std::min(deepest, p.separation);
    const Real supportLevel = std::max(deepest + kEqualDepth, touching);
    std::vector<ContactCandidate> support, approaching;
    for (const ContactCandidate& p : points) (p.separation <= supportLevel ? support : approaching).push_back(p);
    std::stable_sort(approaching.begin(), approaching.end(), [](const ContactCandidate& x, const ContactCandidate& y) { return x.separation < y.separation; });
    support.insert(support.end(), approaching.begin(), approaching.end());
    return support;
}

} // namespace kp
