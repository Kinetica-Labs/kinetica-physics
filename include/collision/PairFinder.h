#pragma once
// Поиск всех пар тел, которые за время h могут сблизиться до касания.

#include <math/StepMemory.h>
#include <collision/ContactPair.h>
#include <collision/broadphase/DynamicTree.h>
#include <joints/JointSet.h>

#include <vector>

namespace kp {

class PairFinder {
public:
    PairFinder(const std::vector<Body>& bodies, Vector3 gravity) : bodies_(bodies), gravity_(gravity) {}
    // Вершина грани, лежащая на границе другой грани с такой точностью, считается оставшейся
    // внутри. У кубов в стопке рёбра совпадают, и без запаса шире дрожи тел вершина то остаётся,
    // то заменяется точкой пересечения почти параллельных рёбер — а та прыгает вдоль ребра.
    Real boundary = kOnBoundary;
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

private:
    StepArray<ContactPair> between(int i, int j, Real h) const;

    const std::vector<Body>& bodies_;
    Vector3 gravity_;
};

} // namespace kp
