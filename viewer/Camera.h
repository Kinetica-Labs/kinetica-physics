#pragma once
// Камера окна просмотра: смотрит на точку target с расстояния distance, вращается вокруг неё.

#include <math/Vector3.h>

#include <array>
#include <cmath>

struct Camera {
    kp::Vector3 target;
    double distance = 5; // м
    double yaw = 0.6;    // поворот вокруг вертикали, рад
    double pitch = 0.3;  // подъём над горизонтом, рад

    static constexpr double kTanHalfFov = 0.5;  // тангенс половины угла обзора по вертикали: обзор около 53°
    static constexpr double kNearPlane = 0.05; // ближе этого к камере ничего не рисуется, м

    kp::Vector3 eye() const {
        return target + distance * kp::Vector3{std::cos(pitch) * std::sin(yaw), std::sin(pitch), std::cos(pitch) * std::cos(yaw)};
    }

    // Единичные оси камеры в мире.
    struct Axes {
        kp::Vector3 right, up, forward;
    };
    Axes axes() const {
        using namespace kp;
        const Vector3 forward = (1 / distance) * (target - eye());
        const Vector3 sideways = forward.cross({0, 1, 0});
        const Vector3 right = (1 / sideways.length()) * sideways;
        return {right, right.cross(forward), forward};
    }

    // Матрица вида 4 × 4 по столбцам, как её ждёт OpenGL: строки — оси камеры «вправо», «вверх»
    // и «назад», последний столбец переносит глаз в начало координат.
    std::array<double, 16> view() const {
        const Axes a = axes();
        const kp::Vector3 from = eye();
        return {a.right.x, a.up.x, -a.forward.x, 0, a.right.y, a.up.y, -a.forward.y, 0, a.right.z, a.up.z, -a.forward.z, 0,
                -a.right.dot(from), -a.up.dot(from), a.forward.dot(from), 1};
    }

    // Направление луча из глаза через пиксель окна width × height — обратное к project.
    kp::Vector3 ray(double x, double y, int width, int height) const {
        const Axes a = axes();
        const double side = (x - 0.5 * width) / (0.5 * height) * kTanHalfFov, rise = (0.5 * height - y) / (0.5 * height) * kTanHalfFov;
        const kp::Vector3 through = a.forward + kp::Real(side) * a.right + kp::Real(rise) * a.up;
        return (1 / through.length()) * through;
    }

    // Пиксель окна width × height, в который попадает точка мира; false — точка позади камеры.
    bool project(kp::Vector3 point, int width, int height, double& x, double& y) const {
        const Axes a = axes();
        const kp::Vector3 offset = point - eye();
        const double depth = a.forward.dot(offset);
        if (depth <= kNearPlane) return false;
        x = 0.5 * width + 0.5 * height * a.right.dot(offset) / (depth * kTanHalfFov);
        y = 0.5 * height - 0.5 * height * a.up.dot(offset) / (depth * kTanHalfFov);
        return true;
    }
};
