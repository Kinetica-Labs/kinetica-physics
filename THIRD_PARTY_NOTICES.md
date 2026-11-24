# Чужой код

В библиотеке движка (`engine/`) чужого кода нет: C++17 и стандартная библиотека.

| Что | Где используется | Версия | Лицензия | Как попадает |
|---|---|---|---|---|
| [GLFW](https://www.glfw.org) — окно и ввод | только `viewer/` | 3.3 и новее | zlib/libpng | установленная в системе (на рабочей машине — пакет `libglfw3-dev` 3.3.6), иначе скачивается версия 3.4 с проверкой SHA-256 (`viewer/CMakeLists.txt`) |
| [Dear ImGui](https://github.com/ocornut/imgui) — панель окна | только `viewer/` | 1.92.7 | MIT | скачивается при сборке с проверкой SHA-256 (`viewer/CMakeLists.txt`) |

Правило: код берётся только из проектов с разрешительной лицензией и записывается сюда; идеи и
алгоритмы из остальных источников переписываются заново по статье.

Для жёсткой тяги, реечной передачи и стенда подвески источником постановки служит Jolt Physics 5.3.0, автор Jorrit Rouwe и участники проекта, лицензия MIT. В Kinetica строки связей выведены для собственного `Bond`. Стенд колеса использует линейную подвеску и собственную модель скольжения с эллиптическим пределом сцепления. [Исходные ограничения Jolt](https://github.com/jrouwe/JoltPhysics/tree/v5.3.0/Jolt/Physics/Constraints) · [Колёсный модуль Jolt](https://github.com/jrouwe/JoltPhysics/tree/v5.3.0/Jolt/Physics/Vehicle) · [Лицензия MIT](https://github.com/jrouwe/JoltPhysics/blob/v5.3.0/LICENSE)

Образец четырёхколёсного шасси опирается на идею виртуальных колёс Jolt 5.3.0. В Kinetica её реализуют собственные `castRay`, `WheelContact` и привод с заданной скоростью обода. [VehicleConstraint в Jolt](https://github.com/jrouwe/JoltPhysics/blob/v5.3.0/Jolt/Physics/Vehicle/VehicleConstraint.cpp)

При выборе обхода дерева пар изучен Box3D Эрина Катто (`be8c73189`, MIT). Обход Kinetica выводится из свойства ближайшего общего предка двух листьев и реализован для существующего двоичного дерева проекта. [Исходный обход Box3D](https://github.com/erincatto/box3d/blob/be8c7318901dbb14b9fbb26fe0a7a48850f53b0e/src/broad_phase.c) · [Лицензия](https://github.com/erincatto/box3d/blob/be8c7318901dbb14b9fbb26fe0a7a48850f53b0e/LICENSE)
