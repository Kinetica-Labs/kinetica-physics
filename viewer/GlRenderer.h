#pragma once
// Рендер на OpenGL. Движок отдаёт точки и отрезки отладочной геометрии; здесь им назначается цвет и
// они рисуются с точки зрения камеры. Закрашенные тела рисуются по формам тел мира. О физике рендер
// не знает ничего.

#include "Camera.h"
#include <debug/DebugGeometry.h>
#include <world/World.h>

#include <string>

// Рисует кадр в текущее окно OpenGL: в прямоугольник width × height пикселей, отступив left пикселей от
// левого края (там боковая панель). filled — мир, тела которого рисуются ещё и закрашенными: один источник
// света и клетка в осях тела, по которой видно вращение; nullptr — только отрезки и точки.
void render(const kp::DebugGeometry& geometry, const Camera& camera, int left, int width, int height, const kp::World* filled = nullptr);

// Сохраняет нарисованный кадр в файл формата PPM.
bool saveScreenshot(const std::string& path, int width, int height);
