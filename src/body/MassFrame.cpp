#include <body/MassFrame.h>

#include <body/PrincipalAxes.h>

namespace kp {

MassFrame MassFrame::of(const ConvexHull& hull, Real density) {
    // Каждая грань веером разбивается на треугольники, каждый треугольник с началом координат —
    // тетраэдр. Объём тетраэдра V = a · (b × c) / 6, его центр — (a + b + c) / 4, а интеграл
    // x xᵀ по нему — V / 20 · (a aᵀ + b bᵀ + c cᵀ + s sᵀ), где s = a + b + c.
    Real volume = 0;
    Vector3 moment;      // ∫ x dV
    Real second[3][3]{};  // ∫ x xᵀ dV
    for (const ConvexHull::Face& face : hull.faces()) {
        for (size_t i = 1; i + 1 < face.loop.size(); ++i) {
            const Vector3 a = hull.vertices()[size_t(face.loop[0])], b = hull.vertices()[size_t(face.loop[i])], c = hull.vertices()[size_t(face.loop[i + 1])];
            const Real v = a.dot(b.cross(c)) / 6;
            volume += v;
            moment += (v / 4) * (a + b + c);
            for (Vector3 p : {a, b, c, a + b + c}) {
                const Real x[3] = {p.x, p.y, p.z};
                for (int r = 0; r < 3; ++r)
                    for (int s = 0; s < 3; ++s) second[r][s] += v / 20 * x[r] * x[s];
            }
        }
    }
    const Vector3 centre = (1 / volume) * moment;
    // Тензор инерции относительно центра масс на единицу плотности: I = tr(S) E − S, где S —
    // интеграл x xᵀ, перенесённый в центр масс.
    const Real c[3] = {centre.x, centre.y, centre.z};
    Real inertia[3][3];
    Real trace = 0;
    for (int r = 0; r < 3; ++r) {
        for (int s = 0; s < 3; ++s) second[r][s] -= volume * c[r] * c[s];
        trace += second[r][r];
    }
    for (int r = 0; r < 3; ++r)
        for (int s = 0; s < 3; ++s) inertia[r][s] = (r == s ? trace : Real(0)) - second[r][s];

    const PrincipalAxes axes = PrincipalAxes::of(inertia);
    MassFrame result{{}, {centre, axes.orientation}, volume};
    if (density > 0)
        result.mass = {1 / (density * volume), {1 / (density * axes.moments.x), 1 / (density * axes.moments.y), 1 / (density * axes.moments.z)}};
    return result;
}

} // namespace kp
