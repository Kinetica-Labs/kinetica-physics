#include <kp/dynamics/event_driven/BodySpaceSolve.h>

#include <kp/math/ProfileMatrix.h>

#include <algorithm>

namespace kp {
namespace {

constexpr Real kBodySpaceGain = 32;     // во сколько раз решение в пространстве тел должно быть дешевле, см. BodySpaceSolve::solve

} // namespace

// Та же система (A + P) x = b, решённая в пространстве тел. A = J W Jᵀ: J — линии, W — обратные
// массы и тензоры инерции тел; P — диагональ слагаемого ближайшей точки, ε Aᵢᵢ. Когда между
// немногими телами много линий — у ведра, вставленного в ведро, пятьсот точек контакта на три
// тела, — матрица A плотная, а ранг её не больше шести на тело: разложение 500 × 500 занимало
// 20 мс. По тождеству Вудбери
//   (P + J W Jᵀ)⁻¹ = P⁻¹ − P⁻¹ J (M + Jᵀ P⁻¹ J)⁻¹ Jᵀ P⁻¹,   M = W⁻¹ — массы и тензоры инерции,
// и решать нужно систему с матрицей K = M + Jᵀ P⁻¹ J размером шесть на тело:
//   K y = Jᵀ P⁻¹ b,   x = P⁻¹ (b − J y).
// y — скорости, которые тела получили бы от импульсов x; K — их массы, утяжелённые «пружинами»
// жёсткости 1 / P вдоль линий. Тела идут в порядке появления в линиях — остров за островом, снизу
// вверх, — и профиль K узок так же, как профиль A. Решение выбирается по цене: cost — сумма
// квадратов ширины профиля в пространстве линий. Возвращает false, если там дешевле.
//
// В последней формуле вычитаются близкие числа, и результат делится на малое P: ошибка округления
// вырастает в миллион раз и даёт скоростям тел ошибку порядка 10⁻¹⁰ м/с — выше допуска решателя
// (шары в ведре от неё дробили шаг на сотни ложных событий). Поэтому решение уточняется: невязка
// b − (A + P) x считается в пространстве линий, где она точна, и та же система решается для неё.
bool BodySpaceSolve::solve(const std::vector<size_t>& active, Real cost, std::vector<Real>& x) const {
    Layout layout;
    if (!lay(active, cost, layout)) return false;
    const std::vector<Real> b = x;
    solveFor(active, layout, x);
    std::vector<Real> residual = residualOf(active, b, x);
    solveFor(active, layout, residual);
    for (size_t p = 0; p < active.size(); ++p) x[p] += residual[p];
    return true;
}

// Тела линий в порядке появления и профиль K; false — в пространстве линий дешевле.
bool BodySpaceSolve::lay(const std::vector<size_t>& active, Real cost, Layout& layout) const {
    std::vector<int>& place = layout.place;
    place.assign(bodies_.size(), -1);
    for (size_t p = 0; p < active.size(); ++p) {
        const ContactSystem::Line& line = system_.lines[active[p]];
        for (int body : {line.a, line.b}) {
            if (bodies_[size_t(body)].isStatic() || place[size_t(body)] >= 0) continue;
            place[size_t(body)] = int(layout.listed.size());
            layout.listed.push_back(body), layout.first.push_back(place[size_t(body)]);
        }
        const int a = place[size_t(line.a)], b = place[size_t(line.b)];
        if (a >= 0 && b >= 0) layout.first[size_t(std::max(a, b))] = std::min(layout.first[size_t(std::max(a, b))], std::min(a, b));
    }
    Real bodyCost = Real(72) * Real(active.size());
    for (size_t q = 0; q < layout.listed.size(); ++q) bodyCost += 6 * Real(6 * (int(q) - layout.first[q] + 1)) * Real(6 * (int(q) - layout.first[q] + 1));
    // Переход оправдан, только когда он дешевле в десятки раз: решения двух путей расходятся на
    // уровне округления, а покоящаяся пирамида кубов чувствительна и к нему — на пути через тела
    // её шаг дробился на десятки ложных событий.
    return kBodySpaceGain * bodyCost < cost;
}

// Массы и тензоры инерции тел в мире: блоки 6 × 6 матрицы K = M + Jᵀ P⁻¹ J.
ProfileMatrix BodySpaceSolve::massMatrix(const Layout& layout) const {
    ProfileMatrix k;
    for (size_t q = 0; q < layout.listed.size(); ++q) {
        for (int d = 0; d < 6; ++d) k.addRow(6 * layout.first[q]);
        const Body& body = bodies_[size_t(layout.listed[q])];
        const int base = 6 * int(q);
        for (int d = 0; d < 3; ++d) k.at(base + d, base + d) = 1 / body.mass.inverseMass;
        // Тензор инерции в мире: R diag(I) Rᵀ, по столбцам.
        const Vector3 moments{1 / body.mass.inverseInertia.x, 1 / body.mass.inverseInertia.y, 1 / body.mass.inverseInertia.z};
        for (int c = 0; c < 3; ++c) {
            const Vector3 column = body.orientation.rotate(moments.scaled(body.orientation.rotateInverse({Real(c == 0), Real(c == 1), Real(c == 2)})));
            const Real entries[3] = {column.x, column.y, column.z};
            for (int r = c; r < 3; ++r) k.at(base + 3 + r, base + 3 + c) = entries[r];
        }
    }
    return k;
}

// Линия как строка J: шесть чисел на одно из двух её тел.
void BodySpaceSolve::rowOf(const ContactSystem::Line& line, int body, Real j[6]) {
    const bool isB = body == line.b;
    const Vector3 linear = isB ? line.linear : -1 * line.linear, angular = isB ? line.angularB : -1 * line.angularA;
    j[0] = linear.x, j[1] = linear.y, j[2] = linear.z, j[3] = angular.x, j[4] = angular.y, j[5] = angular.z;
}

// K y = Jᵀ P⁻¹ b, затем x = P⁻¹ (b − J y); x на входе — b.
void BodySpaceSolve::solveFor(const std::vector<size_t>& active, const Layout& layout, std::vector<Real>& x) const {
    ProfileMatrix k = massMatrix(layout);
    std::vector<Real> y(6 * layout.listed.size(), 0);
    for (size_t p = 0; p < active.size(); ++p) {
        const ContactSystem::Line& line = system_.lines[active[p]];
        const Real stiffness = 1 / (kProximal * system_.diagonal[active[p]] + system_.soft[active[p]]);
        for (int bodyX : {line.a, line.b}) {
            if (layout.place[size_t(bodyX)] < 0) continue;
            Real jx[6];
            rowOf(line, bodyX, jx);
            const int baseX = 6 * layout.place[size_t(bodyX)];
            for (int r = 0; r < 6; ++r) y[size_t(baseX + r)] += stiffness * x[p] * jx[r];
            for (int bodyY : {line.a, line.b}) {
                if (layout.place[size_t(bodyY)] < 0 || layout.place[size_t(bodyY)] > layout.place[size_t(bodyX)]) continue;
                Real jy[6];
                rowOf(line, bodyY, jy);
                const int baseY = 6 * layout.place[size_t(bodyY)];
                for (int r = 0; r < 6; ++r)
                    for (int c = 0; c < 6 && baseY + c <= baseX + r; ++c) k.at(baseX + r, baseY + c) += stiffness * jx[r] * jy[c];
            }
        }
    }
    solveCholesky(k, y);
    for (size_t p = 0; p < active.size(); ++p) {
        const ContactSystem::Line& line = system_.lines[active[p]];
        Real along = 0;
        for (int body : {line.a, line.b}) {
            if (layout.place[size_t(body)] < 0) continue;
            Real j[6];
            rowOf(line, body, j);
            for (int r = 0; r < 6; ++r) along += j[r] * y[size_t(6 * layout.place[size_t(body)] + r)];
        }
        x[p] = (x[p] - along) / (kProximal * system_.diagonal[active[p]] + system_.soft[active[p]]);
    }
}

// Невязка b − (A + P) x в пространстве линий, где она точна.
std::vector<Real> BodySpaceSolve::residualOf(const std::vector<size_t>& active, const std::vector<Real>& b, const std::vector<Real>& x) const {
    const size_t m = active.size();
    std::vector<size_t> position(system_.lines.size(), m);
    for (size_t p = 0; p < m; ++p) position[active[p]] = p;
    std::vector<Real> residual(m);
    for (size_t p = 0; p < m; ++p) {
        residual[p] = b[p] - (kProximal * system_.diagonal[active[p]] + system_.soft[active[p]]) * x[p];
        for (const ContactSystem::Entry& entry : system_.matrix[active[p]])
            if (position[entry.column] < m) residual[p] -= entry.value * x[position[entry.column]];
    }
    return residual;
}

} // namespace kp
