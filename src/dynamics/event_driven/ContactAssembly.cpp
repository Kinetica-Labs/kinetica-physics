#include <math/StepMemory.h>
#include <dynamics/event_driven/ContactAssembly.h>

#include <algorithm>
#include <array>
#include <limits>

namespace kp {
namespace {

constexpr Real kIdle = ContactRows::kIdle;
constexpr Real kSliding = ContactRows::kSliding;

} // namespace

// Порядок неизвестных для разложения: остров за островом, внутри острова пары снизу вверх, у
// пары — её нормали, за ними её трение. Остров — пары, связанные общими подвижными телами;
// неподвижное тело островов не связывает. Между островами в матрице откликов стоят нули, а внутри
// стопки пара связана только с соседними, поэтому в таком порядке матрица блочно-диагональная с
// узкой лентой в каждом блоке — и разложение, которое считает только внутри профиля
// (math/ProfileMatrix.h), тратит время, пропорциональное числу неизвестных, а не его кубу.
StepArray<size_t> ContactAssembly::eliminationOrder(const System& s) const {
    // Острова: тела объединяются по парам; метка острова — наименьший номер его тела.
    StepArray<int> island(state_.bodies.size());
    for (size_t i = 0; i < island.size(); ++i) island[i] = int(i);
    const auto root = [&](int body) {
        while (island[size_t(body)] != body) body = island[size_t(body)] = island[size_t(island[size_t(body)])];
        return body;
    };
    const auto join = [&](int x, int y) {
        if (state_.bodies[size_t(x)].isStatic() || state_.bodies[size_t(y)].isStatic()) return;
        const int a = root(x), b = root(y);
        island[size_t(std::max(a, b))] = std::min(a, b);
    };
    for (const Pair& pair : state_.pairs) join(pair.a, pair.b);
    for (const Bond& bond : state_.bonds) join(bond.a, bond.b);
    const auto islandOf = [&](int a, int b) { return root(state_.bodies[size_t(a)].isStatic() ? b : a); };

    // Блоки неизвестных: пары (уже упорядочены снизу вверх), за ними связи в порядке создания.
    // Устойчивая сортировка по острову сохраняет этот порядок внутри острова.
    struct Block {
        int island;
        bool bond;
        size_t index;
    };
    StepArray<Block> blocks;
    for (size_t p = 0; p < state_.pairs.size(); ++p) blocks.push_back({islandOf(state_.pairs[p].a, state_.pairs[p].b), false, p});
    for (size_t j = 0; j < state_.bonds.size(); ++j) blocks.push_back({islandOf(state_.bonds[j].a, state_.bonds[j].b), true, j});
    std::stable_sort(blocks.begin(), blocks.end(), [](const Block& x, const Block& y) { return x.island < y.island; });

    const size_t n = s.rows.size(), firstBond = n + 3 * s.pairs.size();
    StepArray<size_t> order;
    for (const Block& block : blocks) {
        if (block.bond) {
            order.push_back(firstBond + block.index);
            continue;
        }
        const size_t p = block.index;
        for (size_t row = state_.pairs[p].first; row < state_.pairs[p].last; ++row)
            if (s.rowPlace[row] < n) order.push_back(s.rowPlace[row]);
        if (s.pairPlace[p] < s.pairs.size())
            for (size_t k = 0; k < 3; ++k) order.push_back(n + 3 * s.pairPlace[p] + k);
    }
    return order;
}

// Система задачи: нормали участвующих строк, трение пар, у которых такие строки есть, связи
// сочленений; места строк и пар в ней, матрица отклика, её диагональ и порядок исключения.
ContactAssembly::System ContactAssembly::assemble(const StepArray<Real>& goal) {
    System s;
    for (size_t i = 0; i < state_.rows.size(); ++i)
        if (goal[i] != kIdle) s.rows.push_back(i), s.lines.push_back(state_.normalLine(state_.rows[i]));
    // Начальное предположение: давят точки, где уже есть сила (тёплый старт) или где без неё
    // тела сблизились бы больше допуска. Точка, которой до цели не хватает 10⁻¹² м/с, не давит:
    // у давящей точки скорость обязана сравняться с целью точно, и если тела заперты трением покоя
    // (кубы пирамиды касаются боками и стоят на полу), такому требованию не может ответить никакая
    // сила — решение уходило по этой точке всё дальше с каждым кругом.
    state_.pressing.assign(state_.rows.size(), 0);
    for (size_t i : s.rows) state_.pressing[i] = state_.sum[i] > 0 || state_.relativeSpeed(state_.rows[i], state_.rows[i].normal) < goal[i] - kSolved;
    addFriction(s, goal);
    addBonds(s);
    s.rowPlace.assign(state_.rows.size(), s.rows.size());
    for (size_t i = 0; i < s.rows.size(); ++i) s.rowPlace[s.rows[i]] = i;
    s.pairPlace.assign(state_.pairs.size(), s.pairs.size());
    for (size_t q = 0; q < s.pairs.size(); ++q) s.pairPlace[s.pairs[q]] = q;
    s.matrix = ContactSystem::responseMatrix(state_.bodies, s.lines);
    for (size_t i = 0; i < s.lines.size(); ++i)
        for (const Entry& entry : s.matrix[i])
            if (entry.column == i) s.diagonal.push_back(entry.value);
    s.order = eliminationOrder(s);
    return s;
}

// Три линии трения на пару с участвующими строками. Не скользит пара, которая покоилась в начале
// отрезка; при ударе — любая, пока трения хватает.
void ContactAssembly::addFriction(System& s, const StepArray<Real>& goal) {
    state_.stuck.assign(state_.pairs.size(), 0);
    for (size_t p = 0; p < state_.pairs.size(); ++p) {
        const Pair& pair = state_.pairs[p];
        bool active = false;
        for (size_t i = pair.first; i < pair.last; ++i) active = active || goal[i] != kIdle;
        if (!active || pair.friction == 0) continue;
        s.pairs.push_back(p);
        for (int k = 0; k < 3; ++k) s.lines.push_back(state_.frictionLine(pair, k));
        const Slip& start = pair.startSlip;
        const Real speed = start[0] * start[0] + start[1] * start[1] + pair.reach * pair.reach * start[2] * start[2];
        state_.stuck[p] = !state_.holding || speed < kSliding * kSliding;
    }
}

// Линии связей сочленений. Предел хода, до которого сочленение не дошло, не действует; привод с
// конечной силой за нулевое время импульса не даёт. Сочленение давит на предел, если без его
// силы прошло бы дальше. Пружина с демпфером — линия с податливостью: импульс
// λ = −h (k (e + h v) + c v) даёт конечную скорость v = −γ λ − γ h k e, γ = 1 / (h² k + h c);
// к диагонали добавляется γ, цель — −γ h k e.
void ContactAssembly::addBonds(System& s) {
    for (const Bond& bond : state_.bonds) s.lines.push_back(ContactRows::bondLine(bond));
    state_.bondActive.assign(state_.bonds.size(), 1), state_.bondPressing.assign(state_.bonds.size(), 0);
    state_.saturated.assign(state_.bonds.size(), 0), state_.bondFlips.assign(state_.bonds.size(), 0);
    s.soft.assign(s.lines.size(), 0);
    for (size_t j = 0; j < state_.bonds.size(); ++j)
        if (state_.bonds[j].soft && state_.holdTime > 0) s.soft[s.lines.size() - state_.bonds.size() + j] = 1 / (state_.holdTime * state_.holdTime * state_.bonds[j].stiffness + state_.holdTime * state_.bonds[j].damping);
    for (size_t j = 0; j < state_.bonds.size(); ++j) {
        // За нулевое время пружина и демпфер импульса не дают.
        state_.bondActive[j] = !state_.bonds[j].idle && state_.driveCap(j) > 0 && !(state_.bonds[j].soft && state_.holdTime == 0);
        state_.bondPressing[j] = state_.bonds[j].oneSided && state_.speed(ContactRows::bondLine(state_.bonds[j])) < state_.bondGoal[j];
    }
}

} // namespace kp
