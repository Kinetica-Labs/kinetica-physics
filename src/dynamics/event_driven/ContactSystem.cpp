#include <math/StepMemory.h>
#include <dynamics/event_driven/ContactSystem.h>

#include <algorithm>

namespace kp {

namespace {

// Отклик через два общих тела складывается: одинаковые столбцы строки сливаются. Строка — два куска,
// каждый уже по возрастанию столбца (линии первого тела, затем второго); они сливаются устойчиво,
// как при std::stable_sort, но без обращения к куче. merged — рабочий массив.
void mergeColumns(StepArray<ContactSystem::Entry>& row, StepArray<ContactSystem::Entry>& merged) {
    size_t second = 1;
    while (second < row.size() && !(row[second].column < row[second - 1].column)) ++second;
    merged.clear();
    for (size_t x = 0, y = second; x < second || y < row.size();) {
        const ContactSystem::Entry& entry = y == row.size() || (x < second && !(row[y].column < row[x].column)) ? row[x++] : row[y++];
        if (!merged.empty() && merged.back().column == entry.column) merged.back().value += entry.value;
        else merged.push_back(entry);
    }
    row.assign(merged.begin(), merged.end());
}

} // namespace

// Матрица откликов A = J M⁻¹ Jᵀ: A[i][j] — на сколько изменится скорость вдоль линии i от
// единичного импульса вдоль линии j. Ненулевая, только если у линий есть общее подвижное тело,
// поэтому матрица разреженная и строится по телам: каждое тело связывает лишь свои линии.
ContactSystem::SparseMatrix ContactSystem::responseMatrix(Span<Body> bodies, const StepArray<Line>& lines) {
    // Сколько линий у тела и элементов в строке, известно заранее: память под них берётся один раз (строка
    // в 16 элементов иначе росла пять раз, и построение матрицы стоило четверть кадра стоящего столбика).
    StepArray<size_t> count(bodies.size(), 0);
    for (const Line& line : lines) ++count[size_t(line.a)], ++count[size_t(line.b)];
    StepArray<StepArray<size_t>> onBody(bodies.size());
    for (size_t index = 0; index < bodies.size(); ++index) onBody[index].reserve(count[index]);
    for (size_t i = 0; i < lines.size(); ++i) onBody[size_t(lines[i].a)].push_back(i), onBody[size_t(lines[i].b)].push_back(i);

    SparseMatrix matrix(lines.size());
    for (size_t i = 0; i < lines.size(); ++i) {
        const size_t a = size_t(lines[i].a), b = size_t(lines[i].b);
        matrix[i].reserve((bodies[a].isStatic() ? 0 : count[a]) + (bodies[b].isStatic() ? 0 : count[b]));
    }
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
    StepArray<Entry> merged;
    for (StepArray<Entry>& row : matrix) mergeColumns(row, merged);
    return matrix;
}

} // namespace kp
