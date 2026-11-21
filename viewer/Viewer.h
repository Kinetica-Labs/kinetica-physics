#pragma once
// Состояние окна просмотра: какая сцена идёт, её мир, камера и что показывать.

#include "Camera.h"
#include "ImpulseTool.h"
#include "Scenes.h"
#include <debug/DebugGeometry.h>

#include <string>

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
    // Плечо трения качения, мм (SoftStep::rollingResistance). По умолчанию ноль, как в Bullet и
    // ReactPhysics3D: это сухое трение, и на остром ребре оно держит куб в неустойчивом равновесии
    // (при 1 мм — в окне ±0,3° вместе со сном). Движение гасит сопротивление среды.
    float rollingResistance = 0;
    // Сопротивление среды, 1/с (Body::linearDamping и angularDamping всех тел). Раскачанная цепь при
    // 0,1 теряет 99 % энергии за 33 с, при 0,2 — за 18 с, без него не успокаивается. У сцен с точным
    // ответом выключено: формула — без воздуха.
    float damping = 0.1f;
    int contactUpdates = 4;        // обновлений контакта на кадр (SoftStep::contactUpdates)
    int substeps = 8;              // подшагов на кадр всего (SoftStep::substeps)
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
    // Источники импульсов: взрывы и толчки. Принадлежат окну и смену сцены переживают. placing — номер
    // источника, центр которого поставит следующий щелчок по сцене; −1 — никакой.
    std::vector<ImpulseTool> impulses;
    int placing = -1;
    bool placeAt(kp::Vector3 direction); // ставит центр под курсор; true — щелчок потрачен на это
    void fireAll();                      // пуск всех разовых источников
    int frame = 0;
    // Итог проверки стандартной сцены (StandardScene::verdict) в её момент проверки: пустая строка — как должно.
    bool judged = false;
    std::string verdict;
    // Прибор с замерами считает сцены без воздуха, сна и трения качения, с умолчаниями шага: только при
    // таких настройках итог проверки в окне — тот же, что у прибора. measure ставит их и начинает сцену заново.
    bool asMeasured() const;
    void measure();
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
