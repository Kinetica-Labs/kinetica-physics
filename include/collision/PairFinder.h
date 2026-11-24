#pragma once
// Поиск всех пар тел, которые за время h могут сблизиться до касания.

#include <math/StepMemory.h>
#include <collision/ContactPair.h>
#include <collision/CandidateMemory.h>
#include <collision/ManifoldMemory.h>
#include <collision/broadphase/DynamicTree.h>
#include <joints/JointSet.h>

#include <array>
#include <vector>

namespace kp {

class PairFinder {
public:
    PairFinder(const std::vector<Body>& bodies, Vector3 gravity) : bodies_(bodies), gravity_(gravity) {}
    // Вершина грани, лежащая на границе другой грани с такой точностью, считается оставшейся
    // внутри. У кубов в стопке рёбра совпадают, и без запаса шире дрожи тел вершина то остаётся,
    // то заменяется точкой пересечения почти параллельных рёбер — а та прыгает вдоль ребра.
    Real boundary = kOnBoundary;
    // Память площадок (ManifoldMemory): пара простых тел, чья запись ещё годится, получает площадку
    // из неё, без поиска; записи обновляются каждым вызовом pairs. Без памяти ищется каждая пара.
    ManifoldMemory* memory = nullptr;
    // Пары, которые вызывающему уже известны: {a, b, partA, partB} по возрастанию. Такие пары не
    // проверяются и в ответ не попадают — доиску пар посреди шага (PairReach) нужны только новые, а
    // проверка геометрии пары стоит дороже всего остального поиска. nullptr — искать все. Вместе с
    // памятью площадок не применяется: пропущенная пара выпала бы из неё.
    using Key = std::array<int, 4>;
    const StepArray<Key>* known = nullptr;
    // Память кандидатов (CandidateMemory): пока тела не вышли из своих брусков, дерево заново не строится.
    CandidateMemory* remembered = nullptr;
    // Дерево брусков всех тел; брусок охватывает всё, чего тело может достичь за время h.
    DynamicTree tree(Real h) const;
    // Пары по возрастанию номеров тел. Дерево отбирает кандидатов — тела с пересекающимися
    // брусками; каждого кандидата проверяет examinePair. Ответ тот же, что дал бы перебор всех
    // пар. У составных тел пара — это пара частей: на два тела их может прийтись несколько.
    std::vector<ContactPair> pairs(Real h) const;
    // То же, кроме тел, сочленённых друг с другом. Привязь столкновений не отменяет: она
    // стягивает тела, которые друг друга не касаются, и тело, привязанное к полу, обязано на
    // нём лежать.
    std::vector<ContactPair> pairs(Real h, const JointSet& joints) const;
    // То же в массив вызывающего: у шага с постоянной ценой кадра список пар живёт между обновлениями, и
    // память под него (у тысячи тел — мегабайты) не берётся у кучи заново на каждый поиск.
    void collect(Real h, const JointSet& joints, std::vector<ContactPair>& pairs) const;

private:
    // Пары частей двух тел и записи памяти площадок для следующего обновления.
    struct Found {
        struct Pair {
            ContactPair pair;
            const ManifoldMemory::Entry* carried; // прежняя запись памяти, по которой пара перенесена
            std::ptrdiff_t fresh;                 // или номер новой записи в fresh; −1 — записи нет
        };
        StepArray<Pair> pairs;
        StepArray<ManifoldMemory::Entry> fresh;
    };
    // Пара частей: площадка из памяти, если запись ещё годится, иначе поиск; найденное — в found.
    void part(const Body& a, const Body& b, const Key& key, Real reach, Found& found) const;
    StepArray<std::pair<int, int>> candidates(Real h, const StepArray<Real>& fastest) const;
    Found between(int i, int j, Real h, Real reach) const;
    bool listed(int i, int j, int partA, int partB) const;
    bool carryAll(Real h, const StepArray<Real>& fastest, const StepArray<std::pair<int, int>>& candidates, std::vector<ContactPair>& pairs) const;
    void collect(Real h, std::vector<ContactPair>& pairs) const;

    const std::vector<Body>& bodies_;
    Vector3 gravity_;
};

} // namespace kp
