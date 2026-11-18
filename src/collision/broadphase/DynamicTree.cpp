#include <kp/collision/broadphase/DynamicTree.h>

namespace kp {

void DynamicTree::insert(const Bounds& bounds, int item) {
    const int leaf = int(nodes_.size());
    nodes_.push_back({bounds, -1, -1, -1, item, 0});
    if (root_ < 0) {
        root_ = leaf;
        return;
    }
    // Сосед для нового листа. В узле два выбора: стать соседом самого узла — тогда появится новый
    // родитель с объединённым бруском — или спуститься в одного из потомков, и тогда все узлы на
    // пути вырастут. Берётся то, что прибавляет меньше площади.
    int sibling = root_;
    while (!nodes_[size_t(sibling)].leaf()) {
        const Node& node = nodes_[size_t(sibling)];
        const Real combined = surfaceArea(enclosing(node.bounds, bounds));
        const Real growth = 2 * (combined - surfaceArea(node.bounds)); // на столько вырастет сам узел, если спуститься
        const auto descending = [&](int child) {
            const Node& c = nodes_[size_t(child)];
            const Real grown = surfaceArea(enclosing(c.bounds, bounds));
            return growth + (c.leaf() ? grown : grown - surfaceArea(c.bounds));
        };
        const Real left = descending(node.left), right = descending(node.right);
        if (2 * combined < left && 2 * combined < right) break;
        sibling = left < right ? node.left : node.right;
    }

    const int grandparent = nodes_[size_t(sibling)].parent, parent = int(nodes_.size());
    nodes_.push_back({enclosing(bounds, nodes_[size_t(sibling)].bounds), grandparent, sibling, leaf, -1, nodes_[size_t(sibling)].height + 1});
    nodes_[size_t(sibling)].parent = nodes_[size_t(leaf)].parent = parent;
    replaceChild(grandparent, sibling, parent);

    for (int node = parent; node >= 0; node = nodes_[size_t(node)].parent) {
        refit(node);
        node = balanced(node);
    }
}

// Брусок и высота внутреннего узла — по его потомкам.
void DynamicTree::refit(int node) {
    Node& n = nodes_[size_t(node)];
    const Node& left = nodes_[size_t(n.left)];
    const Node& right = nodes_[size_t(n.right)];
    n.bounds = enclosing(left.bounds, right.bounds);
    n.height = 1 + (left.height > right.height ? left.height : right.height);
}

void DynamicTree::replaceChild(int parent, int from, int to) {
    if (parent < 0) root_ = to;
    else (nodes_[size_t(parent)].left == from ? nodes_[size_t(parent)].left : nodes_[size_t(parent)].right) = to;
}

// Если один потомок узла выше другого больше чем на единицу, высокий поднимается на место узла.
// Возвращает узел, который теперь стоит на этом месте.
int DynamicTree::balanced(int node) {
    const Node& n = nodes_[size_t(node)];
    const int tilt = nodes_[size_t(n.right)].height - nodes_[size_t(n.left)].height;
    if (tilt > 1) return raised(node, n.right, n.left);
    if (tilt < -1) return raised(node, n.left, n.right);
    return node;
}

// Поворот: tall, высокий потомок узла node, занимает его место. Узел становится потомком tall и
// вместо tall получает того из его потомков, что ниже; более высокий остаётся при tall.
int DynamicTree::raised(int node, int tall, int other) {
    const int first = nodes_[size_t(tall)].left, second = nodes_[size_t(tall)].right;
    const bool keepFirst = nodes_[size_t(first)].height > nodes_[size_t(second)].height;
    const int kept = keepFirst ? first : second, moved = keepFirst ? second : first;

    nodes_[size_t(tall)].parent = nodes_[size_t(node)].parent;
    replaceChild(nodes_[size_t(node)].parent, node, tall);
    nodes_[size_t(tall)].left = node, nodes_[size_t(tall)].right = kept;
    nodes_[size_t(node)].parent = tall;
    nodes_[size_t(node)].left = other, nodes_[size_t(node)].right = moved;
    nodes_[size_t(moved)].parent = node;
    refit(node);
    refit(tall);
    return tall;
}

} // namespace kp
