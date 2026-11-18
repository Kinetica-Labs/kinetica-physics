#pragma once
// Линейная система одного решения сил контакта: линии ограничений и матрица их откликов.

#include <kp/body/Body.h>

#include <vector>

namespace kp {

constexpr Real kSolved = Real(1e-10); // м/с: поправка скорости, ниже которой точка считается решённой
// Четыре точки на одной грани дают вырожденную матрицу: силу между ними можно распределить
// по-разному при тех же скоростях, и перебор активного набора на такой матрице может не
// закончиться. Поэтому к задаче добавляется слагаемое, которое удерживает силы вблизи прежних:
// (ε/2) Σ Aᵢᵢ (λᵢ − λᵢ прежняя)². Матрица становится строго положительной, перебор — конечным,
// а повторение решения в чередовании с трением сходится к решению исходной задачи: добавка
// исчезает, когда силы перестают меняться (метод ближайшей точки, Rockafellar, 1976).
constexpr Real kProximal = Real(1e-6);

// Неизвестные одного решения: нормали участвующих строк, за ними по три линии трения на пару,
// за ними связи сочленений.
struct ContactSystem {
    // Линия ограничения: единичный импульс вдоль неё даёт телу b силу linear и момент angularB,
    // телу a — противоположные; скорость вдоль неё —
    // linear · (v_b − v_a) + angularB · ω_b − angularA · ω_a.
    struct Line {
        int a, b;
        Vector3 linear, angularA, angularB;
    };
    struct Entry {
        size_t column;
        Real value;
    };
    using SparseMatrix = std::vector<std::vector<Entry>>; // строка — её ненулевые элементы по возрастанию столбца

    std::vector<size_t> rows, pairs; // номера строк и пар
    std::vector<size_t> rowPlace;    // место строки среди неизвестных; rows.size() — строка не участвует
    std::vector<size_t> pairPlace;   // то же для пар
    std::vector<Line> lines;
    SparseMatrix matrix;             // отклик: скорость вдоль линии i от единичного импульса вдоль j
    std::vector<Real> diagonal;      // диагональ матрицы отклика
    std::vector<Real> soft;          // податливость линии: скорость на единицу импульса; ноль у жёстких
    std::vector<size_t> order;       // линии в том порядке, в каком их исключает разложение

    // Матрица откликов линий по телам.
    static SparseMatrix responseMatrix(const std::vector<Body>& bodies, const std::vector<Line>& lines);
};

} // namespace kp
