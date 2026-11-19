#pragma once
// Сцены с сочленениями, у которых есть точное решение: по ним видно, насколько шаг ошибается.
// У каждой — одна величина, которую можно взять и из мира, и из формулы.

#include <world/World.h>

#include <vector>

struct ExactScene {
    const char* key;   // имя в данных замеров
    const char* title;
    kp::World (*build)();
    kp::Vector3 lookAt; // куда смотреть в окне просмотра
    double distance;
    const char* quantity;     // что сравнивается, с единицей измерения
    double (*measured)(const kp::World& world);
    double (*exact)(double time);
};

const std::vector<ExactScene>& exactScenes();
