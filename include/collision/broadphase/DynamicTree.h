#pragma once
// Динамическое дерево ограничивающих брусков — для поиска тел, которые могут соприкоснуться.
//
// Лист дерева — брусок одного тела; внутренний узел — брусок, охватывающий двух потомков. Запрос
// «что пересекается с этим бруском» спускается только в те узлы, с которыми брусок пересёкся,
// поэтому вместо всех тел просматривает порядка log n.
//
// Новый лист вставляется туда, где суммарная площадь поверхности узлов вырастет меньше всего:
// площадь бруска пропорциональна вероятности, что случайный запрос в него попадёт, и дерево с
// меньшей суммой площадей в среднем отвечает быстрее. После вставки путь к корню выравнивается
// поворотами, как в АВЛ-дереве: высоты потомков любого узла различаются не больше чем на единицу,
// поэтому высота дерева не превышает примерно 1,44 log₂ n.
// Catto, «Dynamic Bounding Volume Hierarchies», Game Developers Conference, 2019.

#include <math/Vector3.h>

#include <vector>

namespace kp {

// Брусок со сторонами вдоль осей мира.
struct Bounds {
    Vector3 lower, upper;
};

inline Bounds enclosing(const Bounds& a, const Bounds& b) {
    return {{a.lower.x < b.lower.x ? a.lower.x : b.lower.x, a.lower.y < b.lower.y ? a.lower.y : b.lower.y, a.lower.z < b.lower.z ? a.lower.z : b.lower.z},
            {a.upper.x > b.upper.x ? a.upper.x : b.upper.x, a.upper.y > b.upper.y ? a.upper.y : b.upper.y, a.upper.z > b.upper.z ? a.upper.z : b.upper.z}};
}

// Касание гранями считается пересечением.
inline bool overlapping(const Bounds& a, const Bounds& b) {
    return a.lower.x <= b.upper.x && b.lower.x <= a.upper.x && a.lower.y <= b.upper.y && b.lower.y <= a.upper.y && a.lower.z <= b.upper.z &&
           b.lower.z <= a.upper.z;
}

inline Real surfaceArea(const Bounds& b) {
    const Vector3 d = b.upper - b.lower;
    return 2 * (d.x * d.y + d.y * d.z + d.z * d.x);
}

class DynamicTree {
public:
    struct Node {
        Bounds bounds;
        int parent = -1, left = -1, right = -1;
        int item = -1;  // у листа — номер того, что в нём лежит
        int height = 0; // у листа ноль
        bool leaf() const { return left < 0; }
    };

    // Место под items листьев сразу: дерево из n листьев — это 2n − 1 узел.
    void reserve(int items) { nodes_.reserve(size_t(2 * items)); }
    void insert(const Bounds& bounds, int item);

    // Вызывает visit(item) для каждого листа, брусок которого пересекается с bounds.
    template <class Visit> void query(const Bounds& bounds, Visit&& visit) const {
        if (root_ < 0) return;
        int stack[kMaxHeight + 1], size = 0; // обход в глубину держит в стеке не больше высоты дерева плюс один узел
        stack[size++] = root_;
        while (size > 0) {
            const Node& node = nodes_[size_t(stack[--size])];
            if (!overlapping(node.bounds, bounds)) continue;
            if (node.leaf()) visit(node.item);
            else stack[size++] = node.left, stack[size++] = node.right;
        }
    }

    int height() const { return root_ < 0 ? 0 : nodes_[size_t(root_)].height; }
    int root() const { return root_; }
    const std::vector<Node>& nodes() const { return nodes_; }

private:
    static constexpr int kMaxHeight = 96; // с запасом: выровненному дереву из 2³² листьев хватает 47

    void refit(int node);
    int balanced(int node);
    int raised(int node, int tall, int other);
    void replaceChild(int parent, int from, int to);

    std::vector<Node> nodes_;
    int root_ = -1;
};

} // namespace kp
