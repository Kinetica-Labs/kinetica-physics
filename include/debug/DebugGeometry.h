#pragma once
// Отладочная геометрия: всё, что движок может показать о себе, он отдаёт точками и отрезками.
//
// Движок ничего не рисует сам и не знает, кто будет рисовать: браузер, OpenGL или печать в
// консоль. Так устроена отладка во всех больших движках — `b3DebugDraw` в Box3D, `btIDebugDraw`
// в Bullet, `DebugRenderer` в Jolt, `PxRenderBuffer` в PhysX. Здесь выбран вариант PhysX: не
// обратные вызовы, а готовый список, потому что список — это обычное возвращаемое значение.
//
// Отладка читает мир снаружи и ничего в нём не меняет: в коде физики нет ни одной её строки.

#include <collision/narrowphase/Gjk.h>
#include <world/World.h>

#include <string>
#include <vector>

namespace kp {

// Что означает точка или отрезок. Цвет выбирает тот, кто рисует.
enum class DebugKind { Shape, StaticShape, ContactPoint, ContactNormal, ClosestPoints, TreeNode };

struct DebugPoint {
    Vector3 position; // мир
    DebugKind kind;
};
struct DebugLine {
    Vector3 from, to; // мир
    DebugKind kind;
};

struct DebugGeometry {
    std::vector<DebugPoint> points;
    std::vector<DebugLine> lines;
    Real normalLength = Real(0.1); // длина отрезка, которым показана нормаль контакта, м

    // Рёбра многогранника; шар и капсула — окружностями.
    DebugGeometry& add(const ConvexShape& shape, const Transform& pose, DebugKind kind = DebugKind::Shape);
    // Точки контакта на теле A и нормаль из каждой.
    DebugGeometry& add(const ContactManifold& manifold);
    // Ближайшие точки двух форм и отрезок между ними.
    DebugGeometry& add(const ClosestPoints& closest);
    // Бруски внутренних узлов дерева поиска пар.
    DebugGeometry& add(const DynamicTree& tree);
};

// Весь мир: формы всех тел и контакты касающихся пар; по желанию — узлы дерева поиска пар.
DebugGeometry debugGeometry(const World& world, bool withTree = false);

// То же словами, для консоли: по строке на тело, на пару в контакте и на последний шаг.
std::string debugText(const World& world);

} // namespace kp
