#pragma once
// Память площадок: точки контакта пары многогранников, найденные раньше, идут за телами вместо
// нового поиска, пока пара почти не сдвинулась (в PhysX — persistent contact manifold).
//
// Контакт по грани — это вершины лежащего тела над опорной гранью другого. Пока тела относительно
// друг друга почти неподвижны, состав точек прежний: каждая едет со своим телом, а её зазор —
// расстояние до плоскости грани, как в свежем поиске. Искать заново нужно, когда к грани могла
// подойти вершина, которой в площадке нет. При поиске такая вершина была дальше запаса
// kContactDistance + reach₀, а с тех пор любая точка меньшего тела сместилась относительно большего
// не больше чем на drift = |Δp| + 2 sin(θ/2) R: Δp — смещение его центра в осях большего, θ — угол
// относительного поворота, R — охватывающий радиус. Запись годится, пока
//     drift + max(0, reach − reach₀) <= tolerance:
// тогда пропущенная вершина к концу обновления войдёт не глубже tolerance − kContactDistance, а
// точка пересечения рёбер уйдёт вдоль ребра не дальше tolerance.

#include <collision/ContactPair.h>
#include <math/StepMemory.h>
#include <math/Transform.h>

#include <optional>
#include <vector>

namespace kp {

class ManifoldMemory {
public:
    static constexpr int kPoints = 8; // площадку крупнее память не ведёт
    struct Entry {
        int a, b;
        const ConvexShape *shapeA, *shapeB; // тело с другим номером или другой формой чужую запись не получит
        Transform apart;                    // поза меньшего тела в осях большего на момент поиска
        Real reach;                         // путь сближения пары на момент поиска, м
        bool faceOnB;
        Vector3 normal, onFace; // нормаль площадки и точка опорной грани — в осях тела с этой гранью
        int count;
        struct Point {
            Vector3 at; // точка лежащего тела — в его осях
            std::uint32_t id;
        } points[kPoints];
    };
    Real tolerance = 0; // м

    // Запись свежей площадки пары простых тел; nullopt — такую площадку память не ведёт: контакт
    // не по грани (одна точка: пара рёбер, ближайшие точки) или тело не многогранник.
    static std::optional<Entry> entryOf(const ContactPair& pair, const Body& a, const Body& b);
    // Запись пары тел i и j, если она есть и при нынешних позах ещё годится; reach — нынешний путь
    // сближения пары.
    const Entry* valid(int i, int j, const Body& a, const Body& b, Real reach) const;
    // Площадка записи в нынешних позах тел.
    static ContactPair carried(const Entry& entry, const Body& a, const Body& b, Real reach);
    // Записи следующего обновления, по возрастанию (a, b).
    void replace(const StepArray<Entry>& entries) { entries_.assign(entries.begin(), entries.end()); }

private:
    std::vector<Entry> entries_;
};

} // namespace kp
