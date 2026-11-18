#pragma once
// Состояние окна просмотра: какая сцена идёт, её мир, камера и что показывать.

#include "Camera.h"
#include "Scenes.h"
#include <kp/debug/DebugGeometry.h>

struct Viewer {
    size_t scene = 0;
    kp::World world;
    Camera camera;
    bool paused = false, stepOnce = false;
    bool show[6] = {true, true, true, true, true, false}; // по одному флагу на DebugKind
    bool showBodyNumbers = true;
    bool fill = true;              // тела закрашены: свет и клетка в осях тела (GlRenderer)
    bool shockPropagation = false; // необязательный проход ударной волны в решателе
    bool sleeping = true;          // покоящиеся острова тел засыпают (World::sleeping)
    int segmentLimit = 8;          // отрезков на кадр точного шага, не больше (World::maxSegments)
    bool realtime = true;          // шаг с постоянной ценой кадра (World::realtime); без него шаг точный
    float rollingResistance = 1;   // плечо трения качения, мм (SoftStep::rollingResistance); 0 — качение без потерь
    // Сколько раз за кадр шаг с постоянной ценой заново ищет точки и нормали контакта (мир шагает
    // kFrame / contactUpdates, подшагов во столько же раз меньше). Нормаль контакта за шаг мира не
    // поворачивается вместе с телами: при одном обновлении на кадр качающаяся цепь из торов набирала
    // энергию (54 → 141 Дж за 30 с), при четырёх — теряет.
    int contactUpdates = 4;
    int substeps = 8;              // подшагов на кадр всего (SoftStep::substeps на шаг мира — substeps / contactUpdates)
    double lag = 0;                // на сколько сцена отстала от часов из-за этого предела, с

    // Захват тела мышью. Луч от курсора находит тело и точку на нём; к точке цепляется упругая
    // привязь (TetherJoint), другой её конец идёт за курсором в плоскости экрана на той же
    // глубине. Привязь у мира одна: отпущенная, она остаётся без жёсткости и ничего не держит.
    struct Grab {
        int body = -1; // −1 — ничего не держим
        kp::TetherJoint* tether = nullptr; // привязь мира; создаётся при первом захвате
        double depth = 0; // расстояние от глаза до точки захвата вдоль взгляда, м
    } grab;
    // Собственная частота тела на привязи, Гц. Жёсткость — масса × (2π · частота)²: при 8 Гц
    // привязь, растянутая на 10 см, тянет с силой в 25 весов тела — хватает, чтобы вытащить куб
    // из-под двух десятков таких же.
    static constexpr double kGrabFrequency = 8;
    bool grabAt(kp::Vector3 direction);
    void dragTo(kp::Vector3 direction);
    void release();
    int frame = 0;
    double worstError = 0, worstGap = 0; // наибольшие ошибка величины с точным решением и разрыв сочленений, м
    double lastStep = 0, slowestStep = 0; // время счёта шага, мс
    int mostSegments = 0;                 // наибольшее число отрезков в одном шаге

    static constexpr kp::Real kFrame = kp::Real(1) / 60; // шаг мира, с

    // Строит сцену заново и наводит на неё камеру.
    void load(size_t index);
    // Продвигает мир на один шаг и замеряет, сколько это заняло.
    void step();
    // Та же геометрия без видов, выключенных галочками.
    // Видимая часть геометрии; к ней добавлена натянутая привязь.
    kp::DebugGeometry visible(const kp::DebugGeometry& all) const;
};
