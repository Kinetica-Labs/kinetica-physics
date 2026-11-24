#pragma once
// Для всех движков: импульсы сухого трения за один шаг при заданной нормальной нагрузке.
// Эталон минимизирует кинетическую энергию на двух дисках Кулона. При косом скольжении
// направление трения в непрерывном времени может меняться в течение шага.
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdio>
#include <limits>
#include <string_view>

namespace oblique {
// В касательной плоскости yz действительная часть — компонента y, мнимая — z.
using YZ = std::complex<double>;
using Pair = std::array<YZ, 2>;
constexpr double radius = 0.05, sphereMass = 1, force = 10, h = 1.0 / 480;
constexpr double speedTolerance = 1e-6, impulseTolerance = 1e-7;
constexpr int standardPreload = 120, settledPreload = 1920;
constexpr double pi = 3.14159265358979323846;
constexpr double jawHalfX = 0.02, jawHalfYZ = 0.4, jawDensity = 500;
constexpr double sphereDensity = 3 * sphereMass / (4 * pi * radius * radius * radius);
inline double jawPosition(int side) { return (side == 0 ? -1 : 1) * (radius + jawHalfX); }

struct Launch { YZ centre, angular; }; // v_y, v_z [м/с] и ω_y, ω_z [рад/с]
inline Launch initialMotion(Pair u) {
    const YZ spin = (u[1] - u[0]) / (2 * radius);
    return {(u[0] + u[1]) / 2.0, {-spin.imag(), spin.real()}};
}
inline Pair poleSpeeds(YZ centre, YZ angular) {
    const YZ spin{radius * angular.imag(), -radius * angular.real()};
    return {centre - spin, centre + spin};
}


inline YZ onDisk(YZ impulse, double limit) {
    const double length = std::abs(impulse);
    return length > limit ? (limit / length) * impulse : impulse;
}

// Импульсы делятся на массу. Диагональ матрицы отклика 1 + mR²/I = 3,5, внедиагональ
// 1 − mR²/I = −1,5. Проекция на диск не растягивает расстояния, поэтому полный проход
// умножает ошибку второго импульса максимум на (1,5/3,5)². Начальная ошибка ограничена
// радиусом его диска (≤ 0,0125 м/с здесь); через 48 проходов погрешность округления преобладает.
inline Pair oracle(Pair u, std::array<double, 2> mu) {
    Pair j{};
    for (int i = 0; i < 48; ++i) {
        j[0] = onDisk((u[0] + 1.5 * j[1]) / 3.5, mu[0] * force * h / sphereMass);
        j[1] = onDisk((u[1] + 1.5 * j[0]) / 3.5, mu[1] * force * h / sphereMass);
    }
    return {u[0] - 3.5 * j[0] + 1.5 * j[1], u[1] - 3.5 * j[1] + 1.5 * j[0]};
}

inline double largestError(double left, double right) {
    if (!std::isfinite(left) || !std::isfinite(right)) return std::numeric_limits<double>::infinity();
    return std::max(left, right);
}

inline void printPair(const Pair& values) {
    for (YZ value : values) std::printf(",%.17g,%.17g", value.real(), value.imag());
}

template<class Clamp>
bool measure(int trial, int angle, Pair u, std::array<double, 2> mu, int preloadSteps) {
    const auto rotation = std::polar(1.0, angle * pi / 180);
    for (auto& part : u) part *= rotation;
    Clamp clamp(mu);
    int preloaded = 0;
    while (preloaded < preloadSteps && clamp.step()) ++preloaded;
    const auto expected = oracle(u, mu);
    clamp.launch(u);
    const auto before = clamp.jawMomentaX();
    const bool stepped = preloaded == preloadSteps && clamp.step();
    const auto after = clamp.jawMomentaX();
    // Импульс реакции из баланса каждой губки вдоль свободной оси её направляющей, Н·с.
    const double normalLeft = force * h - (after[0] - before[0]);
    const double normalRight = force * h + (after[1] - before[1]);
    const auto actual = clamp.poleVelocities(), jaws = clamp.jawVelocities();
    const double slipError = largestError(std::abs(actual[0] - expected[0]), std::abs(actual[1] - expected[1]));
    const double normalError = largestError(std::abs(normalLeft - force * h), std::abs(normalRight - force * h));
    const Pair relative{actual[0] - jaws[0], actual[1] - jaws[1]};
    const double jawError = largestError(std::abs(jaws[0]), std::abs(jaws[1]));
    const double relativeError = largestError(std::abs(relative[0] - expected[0]), std::abs(relative[1] - expected[1]));
    const bool pass = stepped && normalError <= impulseTolerance && slipError <= speedTolerance
                      && jawError <= speedTolerance && relativeError <= speedTolerance;
    std::printf("%d,%d,%.3g,%.3g", trial, angle, mu[0], mu[1]);
    printPair(u);
    printPair(expected);
    printPair(actual);
    printPair(jaws);
    printPair(relative);
    std::printf(",%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%d,%d,%d\n", normalLeft, normalRight, normalError,
                slipError, jawError, relativeError, preloaded, int(stepped), int(pass));
    return pass;
}
template<class Clamp>
int run(int argc, char** argv) {
    const bool settled = argc == 2 && std::string_view(argv[1]) == "--settled";
    if (argc != 1 && !settled) return 2;
    const int preloadSteps = settled ? settledPreload : standardPreload;
    std::puts("trial,angle_deg,mu_left,mu_right,initial_left_y,initial_left_z,initial_right_y,initial_right_z,"
              "oracle_left_y,oracle_left_z,oracle_right_y,oracle_right_z,observed_left_y,observed_left_z,observed_right_y,observed_right_z,"
              "jaw_left_y,jaw_left_z,jaw_right_y,jaw_right_z,relative_left_y,relative_left_z,relative_right_y,relative_right_z,"
              "normal_left_Ns,normal_right_Ns,normal_error_Ns,max_slip_error_m_s,max_jaw_speed_m_s,max_relative_error_m_s,preload_steps,step_complete,pass");
    const Pair initial[] = {{YZ{0.004,0.001}, YZ{-0.003,0.002}}, {YZ{0.08,0.03}, YZ{-0.006,0.011}},
                            {YZ{0.05,-0.04}, YZ{-0.08,0.06}}, {YZ{0,0}, YZ{0.04,0.03}}};
    bool pass = true;
    for (int i = 0; i < 4; ++i) for (int angle : {0, 37, 83})
        for (auto mu : {std::array<double,2>{0.5,0.5}, {0.05,0.6}, {0.6,0.05}, {0,0}})
            pass &= measure<Clamp>(i, angle, initial[i], mu, preloadSteps);
    return pass ? 0 : 1;
}
} // namespace oblique
