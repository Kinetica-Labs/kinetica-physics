#pragma once
// Стандартный набор сцен: то, чем принято испытывать физические движки, — стопка, пирамида, стена
// под ударом, домино, маятник Ньютона, цепь из торов, ведро в ведре, арка, захват робота и другие.
// Сцена — функция, которая строит мир, и проверка того, чем опыт обязан кончиться. Сцены общие
// для окна просмотра (viewer) и для прибора с замерами (validation/studies/StandardScenes.cpp).

#include <world/World.h>

#include <string>
#include <vector>

struct StandardScene {
    const char* key;   // имя в данных замеров
    const char* title;
    kp::World (*build)();
    double seconds;    // сколько времени сцены считать
    double checkAt;    // в какой момент сцена обязана быть в ожидаемом состоянии
    // Чем сцена кончилась: пустая строка — тем, чем должна; иначе — что не так. start — мир в
    // начале опыта, now — в момент checkAt.
    std::string (*verdict)(const kp::World& start, const kp::World& now);
    // Управление приводами по ходу опыта; вызывается перед каждым шагом. Может отсутствовать.
    void (*control)(kp::World& world, double time);
    kp::Vector3 lookAt; // куда смотреть в окне просмотра
    double distance;
};

const std::vector<StandardScene>& standardScenes();

// Показательная постановка на кромку штатной сцены с тремя вёдрами.
kp::World bucketsOnRim();
