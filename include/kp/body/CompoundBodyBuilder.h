#pragma once
// Построение составных тел: из готовых тел, из сетки поверхности, тела вращения, рельеф.

#include <kp/body/Body.h>
#include <kp/collision/shapes/ConvexDecomposition.h>

#include <utility>
#include <vector>

namespace kp {

class CompoundBodyBuilder {
public:
    // Составное тело из готовых тел, стоящих там, где им быть в составе. Масса,
    // центр масс и главные оси инерции находятся по частям; если хоть одна часть неподвижна,
    // неподвижно всё тело. Трение и восстановление берутся у первой части.
    static Body compound(const std::vector<Body>& pieces);

    // Невыпуклое однородное тело по замкнутой сетке его поверхности: сетка разбивается на выпуклые
    // части (collision/shapes/ConvexDecomposition.h), части собираются в составное тело. position — куда
    // поставить начало осей сетки. Масса тела — плотность на объём сетки: лишний объём, который
    // добавляют оболочки частей, на неё не влияет.
    static Body solid(const TriangleMesh& surface, Real density, Vector3 position = {}, const DecompositionLimits& limits = {});

    // Тело вращения вокруг оси Y, заданное точно: profiles — выпуклые многоугольники в полуплоскости
    // (r, y), r >= 0, вместе составляющие профиль тела; segments — число граней по окружности.
    // Каждый многоугольник даёт по выпуклому клину на грань окружности, а если его вращение
    // выпукло целиком (диск дна) — одну часть. В отличие от solid, здесь нет приближения: части
    // в точности повторяют гранёное тело вращения. Тор — один многоугольник-круг, ведро — дно и
    // стенка.
    static Body revolved(const std::vector<std::vector<std::pair<Real, Real>>>& profiles, int segments, Real density, Vector3 position = {});
    // Гранёные тор (ring — радиус осевой окружности, tube — радиус трубки) и ведро донышком вниз
    // (наружные радиусы дна и кромки, высота, толщина стенки и дна); начало осей — центр тора и
    // центр дна ведра снаружи.
    static Body torus(Real ring, Real tube, Real density, Vector3 position = {}, int around = 16, int across = 8);
    static Body bucket(Real bottomRadius, Real topRadius, Real height, Real wall, Real density, Vector3 position = {}, int segments = 16);

    // Неподвижный мир из треугольников: рельеф, пол, стены. Сетка не обязана быть замкнутой.
    // Каждый треугольник — выпуклая призма толщиной thickness, выдавленная внутрь, против его
    // нормали (обход вершин — против часовой стрелки, если смотреть снаружи); призмы собраны в
    // неподвижное составное тело. Толщина нужна только затем, чтобы у призмы был объём: тела
    // останавливает поиск времени удара, а не она.
    static Body ground(const TriangleMesh& surface, Real thickness = Real(0.1));
};

} // namespace kp
