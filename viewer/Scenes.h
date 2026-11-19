#pragma once
// Сцены окна просмотра. Сцена — это функция, которая строит мир, и место, куда смотреть.

#include "../examples/scenes/ExactScenes.h"

#include <world/World.h>

#include <vector>

struct Scene {
    const char* title;
    kp::World (*build)();
    kp::Vector3 lookAt; // точка сцены в середине экрана
    double distance;       // расстояние камеры до неё, м
    // Управление приводами по ходу сцены: вызывается перед каждым шагом. Обычно отсутствует.
    void (*control)(kp::World& world, double time) = nullptr;
    // Сцена с точным решением: в окне видна величина, её точное значение и ошибка.
    const ExactScene* exact = nullptr;
};

const std::vector<Scene>& scenes();
