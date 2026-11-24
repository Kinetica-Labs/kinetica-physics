#include <collision/shapes/ConvexDecomposition.h>

#include <collision/shapes/ConvexHull.h>

#include <algorithm>
#include <limits>
#include <optional>
#include <stdexcept>

namespace kp {
namespace {

constexpr Real kFar = std::numeric_limits<Real>::infinity();

struct Cell {
    int i, j, k;
    int along(int axis) const { return axis == 0 ? i : axis == 1 ? j : k; }
};

// Оболочка облака точек, если оно не плоское.
std::optional<ConvexHull> hullOf(const std::vector<Vector3>& points) {
    if (points.size() < 4) return std::nullopt;
    try {
        return ConvexHull::fromPoints(points);
    } catch (const std::invalid_argument&) {
        return std::nullopt;
    }
}

Real volumeOf(const ConvexHull& hull) {
    Real volume = 0;
    for (const ConvexHull::Face& face : hull.faces())
        for (size_t i = 1; i + 1 < face.loop.size(); ++i)
            volume += hull.vertices()[size_t(face.loop[0])].dot(hull.vertices()[size_t(face.loop[i])].cross(hull.vertices()[size_t(face.loop[i + 1])])) / 6;
    return volume;
}

// Число шагов длины step, которыми покрывается длина: наименьшее целое не меньше length / step.
int stepsAcross(Real length, Real step) {
    const int whole = int(length / step);
    return whole * step < length ? whole + 1 : whole;
}

// Не больше limit вершин оболочки. Оболочка растёт от крайних точек по четырнадцати направлениям;
// каждый раз добавляется вершина, дальше всех выступающая за нынешнюю оболочку, — так при любом
// числе вершин ошибка наименьшая из достижимых жадным выбором.
std::vector<Vector3> trimmed(const ConvexHull& hull, int limit) {
    const std::vector<Vector3>& all = hull.vertices();
    if (int(all.size()) <= limit) return all;
    std::vector<int> chosen;
    for (int d = 0; d < 14; ++d) {
        const Vector3 direction = d < 6 ? Vector3{Real(d == 0) - Real(d == 1), Real(d == 2) - Real(d == 3), Real(d == 4) - Real(d == 5)}
                                     : Vector3{d & 1 ? Real(1) : Real(-1), d & 2 ? Real(1) : Real(-1), d & 4 ? Real(1) : Real(-1)};
        int best = 0;
        for (int v = 1; v < int(all.size()); ++v)
            if (direction.dot(all[size_t(v)]) > direction.dot(all[size_t(best)])) best = v;
        if (std::find(chosen.begin(), chosen.end(), best) == chosen.end()) chosen.push_back(best);
    }
    const auto points = [&] {
        std::vector<Vector3> cloud;
        for (int v : chosen) cloud.push_back(all[size_t(v)]);
        return cloud;
    };
    while (int(chosen.size()) < limit) {
        const std::optional<ConvexHull> current = hullOf(points());
        if (!current) return all;
        int farthest = -1;
        Real reach = 0;
        for (int v = 0; v < int(all.size()); ++v) {
            Real outside = -kFar;
            for (const ConvexHull::Face& face : current->faces()) outside = std::max(outside, face.normal.dot(all[size_t(v)]) - face.offset);
            if (outside > reach) reach = outside, farthest = v;
        }
        if (farthest < 0) break;
        chosen.push_back(farthest);
    }
    return points();
}

class Decomposer {
public:
    Decomposer(const TriangleMesh& solid, const DecompositionLimits& limits) : solid_(solid), limits_(limits) {}
    std::vector<std::vector<Vector3>> run();

private:
    // Область вокселей, которую, возможно, ещё предстоит резать, и насколько она далека от выпуклой.
    struct Region {
        std::vector<Cell> cells;
        Real score = 0; // единица — на пределе допуска
        int depth = 0;
        Vector3 notch;     // точка оболочки, глубже всех ушедшая в пустоту
        bool notched = false;
    };
    // Насколько оболочка выступает в пустоту и где глубже всего.
    struct Gap {
        Real depth = 0;
        Vector3 at;
    };
    struct Part {
        std::vector<Vector3> points;
        Real solidVolume = 0;
        ConvexHull hull;
    };

    bool voxelize();
    void fillColumn(int j, int k, std::vector<Real>& crossings);
    void distanceField();
    std::vector<Vector3> corners(const std::vector<Cell>& cells);
    Gap gapOf(const ConvexHull& hull) const;
    Real scoreOf(const ConvexHull& hull, Real solidVolume, const Gap& gap) const;
    Real scoreOf(const std::vector<Cell>& cells);
    Region region(std::vector<Cell> cells, int depth);
    bool cutThroughNotch(const Region& region, int& axis, int& cut);
    bool emptiestCut(const std::vector<Cell>& cells, int& axis, int& cut) const;
    void split();
    int ownerOf(Vector3 point) const;
    std::vector<std::vector<Vector3>> fitToSurface() const;
    void merge(std::vector<Part>& parts) const;
    std::optional<ConvexHull> mergedHull(const Part& a, const Part& b) const;
    Real mergeScore(const Part& a, const Part& b) const;

    size_t index(int i, int j, int k) const { return size_t(i) + size_t(nx_) * (size_t(j) + size_t(ny_) * size_t(k)); }
    bool inside(int i, int j, int k) const { return i >= 0 && j >= 0 && k >= 0 && i < nx_ && j < ny_ && k < nz_; }
    Vector3 corner(int i, int j, int k) const { return low_ + h_ * Vector3{Real(i), Real(j), Real(k)}; }
    Real volumeOf(const std::vector<Cell>& cells) const { return Real(cells.size()) * h_ * h_ * h_; }
    Real solidVolumeOf(const std::vector<Cell>& cells) const;

    const TriangleMesh& solid_;
    const DecompositionLimits& limits_;
    Vector3 low_;                 // угол сетки вокселей
    Real h_ = 0;               // ребро вокселя, м
    int nx_ = 0, ny_ = 0, nz_ = 0;
    std::vector<char> occupied_;
    std::vector<Real> empty_;  // квадрат расстояния от центра вокселя до ближайшего занятого, в вокселях
    std::vector<Cell> all_;    // занятые воксели
    Real totalVolume_ = 0;
    Real gapTolerance_ = 0;    // м
    std::vector<int> stamp_;   // corners: метки принадлежности вокселя нынешнему набору
    std::vector<int> cornerStamp_;
    int stampId_ = 0;
    std::vector<std::vector<Cell>> leaves_; // почти выпуклые области
    std::vector<int> owner_;   // воксель -> область
};

// Сетка вокселей: resolution ячеек вдоль наибольшего размера. Воксель занят, если его центр
// внутри тела. Внутри ли точка — решает чётность числа пересечений луча вдоль x с поверхностью:
// для каждого столбика (j, k) находятся все пересечения его оси с треугольниками, и воксели
// между нечётным и чётным пересечениями — внутри. Попадание луча в треугольник проверяется в
// проекции правилом «чёт-нечет» с полуоткрытыми отрезками: точка на общем ребре двух
// треугольников засчитывается ровно одному из них.
// Столбик вокселей (j, k) вдоль x: пересечения его оси с поверхностью тела и воксели между
// нечётным и чётным пересечениями. crossings — рабочий массив.
void Decomposer::fillColumn(int j, int k, std::vector<Real>& crossings) {
    const Real y = low_.y + (j + Real(0.5)) * h_, z = low_.z + (k + Real(0.5)) * h_;
    crossings.clear();
    for (const std::array<int, 3>& t : solid_.triangles) {
        const Vector3 p[3] = {solid_.vertices[size_t(t[0])], solid_.vertices[size_t(t[1])], solid_.vertices[size_t(t[2])]};
        bool hit = false;
        for (int e = 0; e < 3; ++e) {
            const Vector3 a = p[e], b = p[(e + 1) % 3];
            if ((a.z > z) != (b.z > z) && y < a.y + (b.y - a.y) * (z - a.z) / (b.z - a.z)) hit = !hit;
        }
        const Vector3 normal = (p[1] - p[0]).cross(p[2] - p[0]);
        if (hit && normal.x != 0) crossings.push_back(p[0].x - (normal.y * (y - p[0].y) + normal.z * (z - p[0].z)) / normal.x);
    }
    std::sort(crossings.begin(), crossings.end());
    for (size_t c = 0; c + 1 < crossings.size(); c += 2)
        for (int i = 0; i < nx_; ++i) {
            const Real x = low_.x + (i + Real(0.5)) * h_;
            if (x > crossings[c] && x < crossings[c + 1]) occupied_[index(i, j, k)] = 1;
        }
}

bool Decomposer::voxelize() {
    if (solid_.vertices.empty() || solid_.triangles.empty()) return false;
    Vector3 high = low_ = solid_.vertices[0];
    for (Vector3 v : solid_.vertices) {
        low_ = {std::min(low_.x, v.x), std::min(low_.y, v.y), std::min(low_.z, v.z)};
        high = {std::max(high.x, v.x), std::max(high.y, v.y), std::max(high.z, v.z)};
    }
    const Vector3 extent = high - low_;
    const Real largest = std::max({extent.x, extent.y, extent.z});
    if (largest <= 0) return false;
    h_ = largest / Real(std::max(limits_.resolution, 4));
    nx_ = std::max(1, stepsAcross(extent.x, h_)), ny_ = std::max(1, stepsAcross(extent.y, h_)), nz_ = std::max(1, stepsAcross(extent.z, h_));
    occupied_.assign(size_t(nx_) * size_t(ny_) * size_t(nz_), 0);

    std::vector<Real> crossings;
    for (int k = 0; k < nz_; ++k)
        for (int j = 0; j < ny_; ++j) fillColumn(j, k, crossings);
    for (int k = 0; k < nz_; ++k)
        for (int j = 0; j < ny_; ++j)
            for (int i = 0; i < nx_; ++i)
                if (occupied_[index(i, j, k)]) all_.push_back({i, j, k});
    if (all_.empty()) return false;
    totalVolume_ = volumeOf(all_);
    gapTolerance_ = std::max(limits_.gap * extent.length(), h_); // мельче вокселя сетка не различает
    stamp_.assign(occupied_.size(), 0);
    cornerStamp_.assign(size_t(nx_ + 1) * size_t(ny_ + 1) * size_t(nz_ + 1), 0);
    distanceField();
    return true;
}

// Квадрат расстояния, в вокселях, от центра каждого вокселя до центра ближайшего занятого: точное
// евклидово преобразование расстояний — вдоль каждой прямой берётся нижняя огибающая парабол
// (q − v)² + f(v), прямые идут вдоль x, затем y, затем z. Felzenszwalb, Huttenlocher, «Distance
// transforms of sampled functions», Theory of Computing 8(19), 2012.
void Decomposer::distanceField() {
    constexpr Real big = Real(1e30);
    std::vector<Real> f, bound;
    std::vector<int> vertex;
    const auto line = [&](size_t first, size_t stride, int n) {
        f.resize(size_t(n)), vertex.resize(size_t(n)), bound.resize(size_t(n) + 1);
        for (int q = 0; q < n; ++q) f[size_t(q)] = empty_[first + stride * size_t(q)];
        const auto meet = [&](int q, int r) { return ((f[size_t(q)] + Real(q) * q) - (f[size_t(r)] + Real(r) * r)) / (2 * Real(q - r)); };
        int k = 0;
        vertex[0] = 0, bound[0] = -big, bound[1] = big;
        for (int q = 1; q < n; ++q) {
            Real s = meet(q, vertex[size_t(k)]);
            while (k > 0 && s <= bound[size_t(k)]) s = meet(q, vertex[size_t(--k)]);
            ++k;
            vertex[size_t(k)] = q, bound[size_t(k)] = s, bound[size_t(k) + 1] = big;
        }
        k = 0;
        for (int q = 0; q < n; ++q) {
            while (bound[size_t(k) + 1] < q) ++k;
            const Real apart = Real(q - vertex[size_t(k)]);
            empty_[first + stride * size_t(q)] = std::min(apart * apart + f[size_t(vertex[size_t(k)])], big);
        }
    };
    empty_.resize(occupied_.size());
    for (size_t c = 0; c < occupied_.size(); ++c) empty_[c] = occupied_[c] ? Real(0) : big;
    for (int k = 0; k < nz_; ++k)
        for (int j = 0; j < ny_; ++j) line(index(0, j, k), 1, nx_);
    for (int k = 0; k < nz_; ++k)
        for (int i = 0; i < nx_; ++i) line(index(i, 0, k), size_t(nx_), ny_);
    for (int j = 0; j < ny_; ++j)
        for (int i = 0; i < nx_; ++i) line(index(i, j, 0), size_t(nx_) * size_t(ny_), nz_);
}

// Углы вокселей набора, которые могут оказаться вершинами его оболочки: углы вокселей, у
// которых хоть одна грань открыта внутри набора. Общий угол соседних вокселей выдаётся один раз.
std::vector<Vector3> Decomposer::corners(const std::vector<Cell>& cells) {
    ++stampId_;
    for (const Cell& c : cells) stamp_[index(c.i, c.j, c.k)] = stampId_;
    const auto member = [&](int i, int j, int k) { return inside(i, j, k) && stamp_[index(i, j, k)] == stampId_; };
    std::vector<Vector3> points;
    for (const Cell& c : cells) {
        if (member(c.i + 1, c.j, c.k) && member(c.i - 1, c.j, c.k) && member(c.i, c.j + 1, c.k) && member(c.i, c.j - 1, c.k) && member(c.i, c.j, c.k + 1) &&
            member(c.i, c.j, c.k - 1))
            continue;
        for (int n = 0; n < 8; ++n) {
            const int i = c.i + (n & 1), j = c.j + ((n >> 1) & 1), k = c.k + ((n >> 2) & 1);
            int& seen = cornerStamp_[size_t(i) + size_t(nx_ + 1) * (size_t(j) + size_t(ny_ + 1) * size_t(k))];
            if (seen == stampId_) continue;
            seen = stampId_;
            points.push_back(corner(i, j, k));
        }
    }
    return points;
}

// Насколько оболочка выступает в пустоту: наибольшее расстояние от её поверхности до тела. Это то,
// что чувствует другое тело: оболочка может добавить много объёма тонким слоем, которого не
// коснуться, или мало — хордой поперёк отверстия кольца, на которой повиснет звено цепи.
// Поверхность оболочки просматривается с шагом в воксель; ближе половины диагонали вокселя к телу
// — значит, касается его.
Decomposer::Gap Decomposer::gapOf(const ConvexHull& hull) const {
    Gap worst;
    const auto visit = [&](Vector3 x) {
        const Vector3 g = (1 / h_) * (x - low_);
        const int i = std::clamp(int(g.x), 0, nx_ - 1), j = std::clamp(int(g.y), 0, ny_ - 1), k = std::clamp(int(g.z), 0, nz_ - 1);
        const Real depth = sqrt(empty_[index(i, j, k)]) * h_ - Real(0.87) * h_;
        if (depth > worst.depth) worst = {depth, x};
    };
    for (const ConvexHull::Face& face : hull.faces()) {
        for (size_t t = 1; t + 1 < face.loop.size(); ++t) {
            const Vector3 a = hull.vertices()[size_t(face.loop[0])], b = hull.vertices()[size_t(face.loop[t])], c = hull.vertices()[size_t(face.loop[t + 1])];
            const Real longest = std::max({(b - a).length(), (c - a).length(), (c - b).length()});
            const int n = std::clamp(stepsAcross(longest, h_), 1, 16);
            for (int u = 0; u <= n; ++u)
                for (int w = 0; u + w <= n; ++w) visit(a + (Real(u) / n) * (b - a) + (Real(w) / n) * (c - a));
        }
    }
    return worst;
}

// Объём тела в области вокселей, с которым сравнивается объём её оболочки. Наклонная поверхность
// в вокселях — лесенка, и оболочка, натянутая на углы ступенек, больше самих вокселей, даже когда
// тело выпукло: повёрнутый брусок из-за этого резался на пять частей. На каждую открытую грань
// вокселя оболочка добавляет не больше половины его объёма, и эта доля прощается.
Real Decomposer::solidVolumeOf(const std::vector<Cell>& cells) const {
    size_t open = 0;
    for (const Cell& c : cells)
        for (int n = 0; n < 6; ++n) {
            const int i = c.i + (n == 0) - (n == 1), j = c.j + (n == 2) - (n == 3), k = c.k + (n == 4) - (n == 5);
            open += !inside(i, j, k) || !occupied_[index(i, j, k)];
        }
    return volumeOf(cells) + Real(open) * h_ * h_ * h_ / 2;
}

// Насколько часть далека от выпуклой; единица — на пределе: большее из «выступ в пустоту к
// допуску» и «лишний объём к его бюджету».
Real Decomposer::scoreOf(const ConvexHull& hull, Real solidVolume, const Gap& gap) const {
    const Real budget = std::max(limits_.concavity * totalVolume_, Real(1e-30));
    return std::max(gap.depth / gapTolerance_, (kp::volumeOf(hull) - solidVolume) / budget);
}

Real Decomposer::scoreOf(const std::vector<Cell>& cells) {
    const std::optional<ConvexHull> hull = hullOf(corners(cells));
    return hull ? scoreOf(*hull, solidVolumeOf(cells), gapOf(*hull)) : Real(0);
}

Decomposer::Region Decomposer::region(std::vector<Cell> cells, int depth) {
    Region result;
    if (const std::optional<ConvexHull> hull = hullOf(corners(cells))) {
        const Gap gap = gapOf(*hull);
        result.score = scoreOf(*hull, solidVolumeOf(cells), gap);
        result.notch = gap.at, result.notched = gap.depth > gapTolerance_;
    }
    result.cells = std::move(cells), result.depth = depth;
    return result;
}

// Разрез области, выступающей в пустоту, — через её выемку, вдоль той оси, при которой обе
// половины выходят ближе всего к выпуклым.
bool Decomposer::cutThroughNotch(const Region& region, int& axis, int& cut) {
    axis = -1;
    Real least = kFar;
    const Vector3 g = (1 / h_) * (region.notch - low_);
    for (int candidate = 0; candidate < 3; ++candidate) {
        const int at = int((candidate == 0 ? g.x : candidate == 1 ? g.y : g.z) + Real(0.5));
        std::vector<Cell> lower, upper;
        for (const Cell& c : region.cells) (c.along(candidate) < at ? lower : upper).push_back(c);
        if (lower.empty() || upper.empty()) continue;
        const Real cost = scoreOf(lower) + scoreOf(upper);
        if (cost < least) least = cost, axis = candidate, cut = at;
    }
    return axis >= 0;
}

// Разрез вдоль осей, после которого в брусках, охватывающих половины, меньше всего пустоты.
bool Decomposer::emptiestCut(const std::vector<Cell>& cells, int& axis, int& cut) const {
    axis = -1;
    Real least = kFar;
    for (int candidate = 0; candidate < 3; ++candidate) {
        int low = cells[0].along(candidate), high = low;
        for (const Cell& c : cells) low = std::min(low, c.along(candidate)), high = std::max(high, c.along(candidate));
        for (int at = low + 1; at <= high; ++at) {
            int lowest[2][3] = {}, highest[2][3] = {}, count[2] = {0, 0};
            for (const Cell& c : cells) {
                const int side = c.along(candidate) < at ? 0 : 1;
                for (int d = 0; d < 3; ++d) {
                    lowest[side][d] = count[side] ? std::min(lowest[side][d], c.along(d)) : c.along(d);
                    highest[side][d] = count[side] ? std::max(highest[side][d], c.along(d)) : c.along(d);
                }
                ++count[side];
            }
            if (!count[0] || !count[1]) continue;
            Real cost = 0;
            for (int side = 0; side < 2; ++side) {
                Real box = 1;
                for (int d = 0; d < 3; ++d) box *= Real(highest[side][d] - lowest[side][d] + 1);
                cost += box - Real(count[side]);
            }
            if (cost < least) least = cost, axis = candidate, cut = at;
        }
    }
    return axis >= 0;
}

// Области режутся начиная с самой невыпуклой; область готова, когда её мера не больше единицы.
// Когда бюджет частей кончается, целыми остаются самые выпуклые области.
void Decomposer::split() {
    std::vector<Region> open;
    open.push_back(region(all_, 0));
    while (!open.empty()) {
        size_t worst = 0;
        for (size_t q = 1; q < open.size(); ++q)
            if (open[q].score > open[worst].score) worst = q;
        Region r = std::move(open[worst]);
        open.erase(open.begin() + std::ptrdiff_t(worst));
        const bool budgetLeft = int(leaves_.size() + open.size()) + 2 <= limits_.maxParts * 2;
        int axis = -1, cut = 0;
        const bool divisible = r.score > 1 && budgetLeft && r.depth < limits_.maxDepth && r.cells.size() >= 8 &&
                               ((r.notched && cutThroughNotch(r, axis, cut)) || emptiestCut(r.cells, axis, cut));
        if (!divisible) {
            leaves_.push_back(std::move(r.cells));
            continue;
        }
        std::vector<Cell> lower, upper;
        for (const Cell& c : r.cells) (c.along(axis) < cut ? lower : upper).push_back(c);
        open.push_back(region(std::move(lower), r.depth + 1));
        open.push_back(region(std::move(upper), r.depth + 1));
    }
}

// Область, которой принадлежит воксель под точкой, или ближайший соседний воксель с хозяином.
int Decomposer::ownerOf(Vector3 point) const {
    const Vector3 g = (1 / h_) * (point - low_);
    const int i = std::clamp(int(g.x), 0, nx_ - 1), j = std::clamp(int(g.y), 0, ny_ - 1), k = std::clamp(int(g.z), 0, nz_ - 1);
    if (owner_[index(i, j, k)] >= 0) return owner_[index(i, j, k)];
    int best = -1;
    Real nearest = kFar;
    for (int dk = -1; dk <= 1; ++dk)
        for (int dj = -1; dj <= 1; ++dj)
            for (int di = -1; di <= 1; ++di) {
                if (!inside(i + di, j + dj, k + dk) || owner_[index(i + di, j + dj, k + dk)] < 0) continue;
                const Vector3 apart = corner(i + di, j + dj, k + dk) + Real(0.5) * Vector3{h_, h_, h_} - point;
                if (apart.dot(apart) < nearest) nearest = apart.dot(apart), best = owner_[index(i + di, j + dj, k + dk)];
            }
    return best;
}

// Подгонка к настоящей поверхности: точки для оболочки части — точки исходной поверхности в её
// области (с шагом в полвокселя) и углы её вокселей на разрезах. Угол на разрезе — тот, вокруг
// которого заняты все восемь вокселей: иначе углы у внутренней кромки дна чашки торчали бы в её
// полость на воксель.
std::vector<std::vector<Vector3>> Decomposer::fitToSurface() const {
    std::vector<std::vector<Vector3>> points(leaves_.size());
    for (const std::array<int, 3>& t : solid_.triangles) {
        const Vector3 a = solid_.vertices[size_t(t[0])], b = solid_.vertices[size_t(t[1])], c = solid_.vertices[size_t(t[2])];
        const Real longest = std::max({(b - a).length(), (c - a).length(), (c - b).length()});
        const int n = std::max(1, stepsAcross(longest, h_ / 2));
        for (int u = 0; u <= n; ++u)
            for (int w = 0; u + w <= n; ++w) {
                const Vector3 x = a + (Real(u) / n) * (b - a) + (Real(w) / n) * (c - a);
                const int owner = ownerOf(x);
                if (owner >= 0) points[size_t(owner)].push_back(x);
            }
    }
    const auto solidCorner = [&](int i, int j, int k) {
        for (int n = 0; n < 8; ++n)
            if (!inside(i - (n & 1), j - ((n >> 1) & 1), k - ((n >> 2) & 1)) || !occupied_[index(i - (n & 1), j - ((n >> 1) & 1), k - ((n >> 2) & 1))]) return false;
        return true;
    };
    for (size_t leaf = 0; leaf < leaves_.size(); ++leaf)
        for (const Cell& c : leaves_[leaf])
            for (int n = 0; n < 8; ++n) {
                const int i = c.i + (n & 1), j = c.j + ((n >> 1) & 1), k = c.k + ((n >> 2) & 1);
                if (solidCorner(i, j, k)) points[leaf].push_back(corner(i, j, k));
            }
    return points;
}

// Наименьший набор: жадно сливается пара соседей, слитая оболочка которых ближе всего к выпуклой,
// — пока она в допуске, а сверх предела числа частей — какой бы она ни была.
// Оболочка двух частей вместе.
std::optional<ConvexHull> Decomposer::mergedHull(const Part& a, const Part& b) const {
    std::vector<Vector3> both = a.points;
    both.insert(both.end(), b.points.begin(), b.points.end());
    return hullOf(both);
}

// Цена слияния двух частей (scoreOf); kFar — не соседи или общей оболочки нет.
Real Decomposer::mergeScore(const Part& a, const Part& b) const {
    const auto boundsOf = [](const Part& part, Vector3& low, Vector3& high) {
        low = high = part.points[0];
        for (Vector3 p : part.points) {
            low = {std::min(low.x, p.x), std::min(low.y, p.y), std::min(low.z, p.z)};
            high = {std::max(high.x, p.x), std::max(high.y, p.y), std::max(high.z, p.z)};
        }
    };
    Vector3 lowI, highI, lowJ, highJ;
    boundsOf(a, lowI, highI), boundsOf(b, lowJ, highJ);
    // Только соседи: бруски частей, раздвинутые на воксель, пересекаются.
    if (lowI.x - h_ > highJ.x || lowJ.x > highI.x + h_ || lowI.y - h_ > highJ.y || lowJ.y > highI.y + h_ || lowI.z - h_ > highJ.z || lowJ.z > highI.z + h_) return kFar;
    const std::optional<ConvexHull> hull = mergedHull(a, b);
    return hull ? scoreOf(*hull, a.solidVolume + b.solidVolume, gapOf(*hull)) : kFar;
}

void Decomposer::merge(std::vector<Part>& parts) const {
    const int n = int(parts.size());
    const auto merged = [&](int i, int j) { return mergedHull(parts[size_t(i)], parts[size_t(j)]); };
    const auto pairScore = [&](int i, int j) { return mergeScore(parts[size_t(i)], parts[size_t(j)]); };
    std::vector<Real> score(size_t(n) * size_t(n), kFar);
    std::vector<char> alive(size_t(n), 1);
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j) score[size_t(i) * size_t(n) + size_t(j)] = pairScore(i, j);
    for (int count = n;; --count) {
        int bi = -1, bj = -1;
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j)
                if (alive[size_t(i)] && alive[size_t(j)] && (bi < 0 || score[size_t(i) * size_t(n) + size_t(j)] < score[size_t(bi) * size_t(n) + size_t(bj)])) bi = i, bj = j;
        if (bi < 0) break;
        const Real best = score[size_t(bi) * size_t(n) + size_t(bj)];
        if (best == kFar || (count <= limits_.maxParts && best > 1)) break;
        parts[size_t(bi)].hull = *merged(bi, bj);
        parts[size_t(bi)].solidVolume += parts[size_t(bj)].solidVolume;
        parts[size_t(bi)].points = parts[size_t(bi)].hull.vertices();
        alive[size_t(bj)] = 0;
        for (int k = 0; k < n; ++k)
            if (k != bi && alive[size_t(k)]) score[size_t(std::min(bi, k)) * size_t(n) + size_t(std::max(bi, k))] = pairScore(std::min(bi, k), std::max(bi, k));
    }
    std::vector<Part> kept;
    for (int i = 0; i < n; ++i)
        if (alive[size_t(i)]) kept.push_back(std::move(parts[size_t(i)]));
    parts = std::move(kept);
}

std::vector<std::vector<Vector3>> Decomposer::run() {
    std::vector<std::vector<Vector3>> result;
    if (!voxelize()) return result;
    split();
    owner_.assign(occupied_.size(), -1);
    for (size_t leaf = 0; leaf < leaves_.size(); ++leaf)
        for (const Cell& c : leaves_[leaf]) owner_[index(c.i, c.j, c.k)] = int(leaf);
    const std::vector<std::vector<Vector3>> points = fitToSurface();
    std::vector<Part> parts;
    for (size_t leaf = 0; leaf < leaves_.size(); ++leaf) {
        const std::optional<ConvexHull> hull = hullOf(points[leaf]);
        if (!hull) continue;
        // Оболочка объединения — оболочка вершин оболочек, поэтому дальше нужны только вершины.
        parts.push_back({hull->vertices(), solidVolumeOf(leaves_[leaf]), *hull});
    }
    merge(parts);
    for (const Part& part : parts) result.push_back(trimmed(part.hull, std::max(limits_.maxHullVertices, 8)));
    return result;
}

} // namespace

std::vector<std::vector<Vector3>> convexParts(const TriangleMesh& solid, const DecompositionLimits& limits) { return Decomposer(solid, limits).run(); }

} // namespace kp
