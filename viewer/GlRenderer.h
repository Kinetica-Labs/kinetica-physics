#pragma once
// Рендер на OpenGL. Движок отдаёт точки и отрезки отладочной геометрии; здесь им назначается цвет и
// они рисуются с точки зрения камеры. Закрашенные тела рисуются по формам тел мира. О физике рендер
// не знает ничего.

#include "Camera.h"
#include <kp/debug/DebugGeometry.h>
#include <kp/world/World.h>

#include <string>

// Рисует кадр в текущее окно OpenGL размером width × height пикселей. filled — мир, тела которого
// рисуются ещё и закрашенными: один источник света и клетка в осях тела, по которой видно вращение;
// nullptr — только отрезки и точки.
void render(const kp::DebugGeometry& geometry, const Camera& camera, int width, int height, const kp::World* filled = nullptr);

// Сохраняет нарисованный кадр в файл формата PPM.
bool saveScreenshot(const std::string& path, int width, int height);
