#pragma once
// Сцены окна просмотра. Сцена — это функция, которая строит мир, и место, куда смотреть.

#include "../examples/scenes/ExactScenes.h"
#include "../examples/scenes/StandardScenes.h"

#include <world/World.h>
#include <world/VehicleConstraint.h>

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
    // Сцена стандартного набора: она знает, чем обязана кончиться, и панель показывает итог проверки.
    const StandardScene* standard = nullptr;
    const char* group = "основные"; // под каким заголовком сцена стоит в панели
    const char* key = nullptr; // имя для --scene
    int updatesPerFrame = 1; // число повторных вычислений внешней силы и шагов мира за 1/60 с
    void (*drawReference)(const kp::World&) = nullptr;
    bool singleSubstep = false; // один поиск контактов и один подшаг за каждый вызов World::step
    const kp::VehicleConstraint* vehicle = nullptr;
};

const std::vector<Scene>& scenes();
size_t sceneIndex(const char* selection); // имя --scene или номер, начиная с 1
