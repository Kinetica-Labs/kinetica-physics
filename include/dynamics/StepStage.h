#pragma once
// Стадии шага мира. Наблюдатель (World::observer) получает управление после каждой: по ним приборы
// считают время и энергию, а физика о приборах не знает.

#include <functional>

namespace kp {

// Точный шаг, на каждом отрезке по порядку: Separation, Impact, Forces, Motion.
// Быстрый шаг: Pairs — поиск пар и точек контакта; Contacts — постройка контактов и строк сочленений;
// затем в каждом подшаге WarmStart, Joints, Normals, Recheck (доиск пар и пропущенных контактов),
// Motion, снова Joints и Normals (без поправки положения), Friction; в конце Memory — силы в память.
// В быстром шаге скорости живут в решателе и попадают в тела в конце шага: посреди него наблюдателю
// верны только позы.
enum class StepStage { Separation, Impact, Forces, Motion, Pairs, Contacts, WarmStart, Joints, Normals, Recheck, Friction, Memory };

using StageObserver = std::function<void(StepStage)>;
inline void notify(const StageObserver& observer, StepStage stage) {
    if (observer) observer(stage);
}

} // namespace kp
