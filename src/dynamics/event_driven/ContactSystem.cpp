#include <math/StepMemory.h>
#include <dynamics/event_driven/ContactSystem.h>

#include <algorithm>

namespace kp {

// Матрица откликов A = J M⁻¹ Jᵀ: A[i][j] — на сколько изменится скорость вдоль линии i от
// единичного импульса вдоль линии j. Ненулевая, только если у линий есть общее подвижное тело,
// поэтому матрица разреженная и строится по телам: каждое тело связывает лишь свои линии.
ContactSystem::SparseMatrix ContactSystem::responseMatrix(const std::vector<Body>& bodies, const StepArray<Line>& lines) {
    StepArray<StepArray<size_t>> onBody(bodies.size());
    for (size_t i = 0; i < lines.size(); ++i) onBody[size_t(lines[i].a)].push_back(i), onBody[size_t(lines[i].b)].push_back(i);

    SparseMatrix matrix(lines.size());
    for (size_t index = 0; index < bodies.size(); ++index) {
        const Body& body = bodies[index];
        if (body.isStatic()) continue;
        for (size_t j : onBody[index]) {
            // Единичный импульс вдоль линии: тело b получает +linear и +angularB, тело a — −linear
            // и −angularA. Отклик тела — изменение его скорости и угловой скорости.
            const Line& source = lines[j];
            const bool sourceIsB = source.b == int(index);
            const Real sign = sourceIsB ? Real(1) : Real(-1);
            const Vector3 velocity = (sign * body.mass.inverseMass) * source.linear;
            const Vector3 spin = sign * inverseInertiaTimes(body, sourceIsB ? source.angularB : source.angularA);
            for (size_t i : onBody[index]) {
                const Line& target = lines[i];
                const bool targetIsB = target.b == int(index);
                const Real change = target.linear.dot(velocity) + (targetIsB ? target.angularB : target.angularA).dot(spin);
                matrix[i].push_back({j, targetIsB ? change : -change});
            }
        }
    }
    // Отклик через два общих тела складывается: одинаковые столбцы строки сливаются.
    for (StepArray<Entry>& row : matrix) {
        std::stable_sort(row.begin(), row.end(), [](const Entry& x, const Entry& y) { return x.column < y.column; });
        size_t kept = 0;
        for (const Entry& entry : row) {
            if (kept > 0 && row[kept - 1].column == entry.column) row[kept - 1].value += entry.value;
            else row[kept++] = entry;
        }
        row.resize(kept);
    }
    return matrix;
}

} // namespace kp
