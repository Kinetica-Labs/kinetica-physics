// Два выпуклых тела, предписанные позы за 1 с. Опыт измеряет CCD без решателя сил.
// Оракул выводится из координат заданных вершин, отдельно для физического нуля и полосы CCD.
#include "HullSweepShapes.h"
#include <collision/ContactPair.h>
#include <collision/narrowphase/Gjk.h>
#include <collision/narrowphase/Sat.h>
#include <collision/narrowphase/TimeOfImpact.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

namespace {
using namespace kp;
constexpr double duration = 1.0, timeTolerance = 2e-7, gapTolerance = 2e-8;
const double absent = std::numeric_limits<double>::quiet_NaN();

Quaternion turnX(double angle) {
    return {Real(std::cos(angle/2)), Real(std::sin(angle/2)), 0, 0};
}

struct Geometry {
    Vector3 centre{}, edge{}, tip{};
    double radius = 0, peakAngle = 0, sweep = .38;
    Geometry(const hull_sweep::Mesh& a, const hull_sweep::Mesh& b) {
        const auto& cap = a.faces[1];
        for (int i : cap) centre += a.vertices[i];
        centre = (Real(1) / Real(cap.size())) * centre;
        edge = Real(.5) * (a.vertices[cap[0]] + a.vertices[cap[1]]);
        // Минимальное расстояние до остальных граничных плоскостей.
        double room = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < a.faces.size(); ++i) {
            if (i == 1) continue;
            const auto& f = a.faces[i];
            const Vector3 p = a.vertices[f[0]];
            const Vector3 n = (a.vertices[f[1]] - p).cross(a.vertices[f[2]] - p);
            room = std::min(room, double(n.dot(p - centre) / n.length()));
        }
        size_t selected = 0;
        for (size_t i = 0; i < b.vertices.size(); ++i) {
            const auto p = b.vertices[i];
            const double r = std::hypot(p.y, p.z);
            if (r > radius) { radius = r; selected = i; }
        }
        tip = b.vertices[selected];
        peakAngle = std::atan2(-tip.y, -tip.z);
        const Quaternion peak = turnX(peakAngle);
        // z_i - z_tip = dz cos(theta) + dy sin(theta).
        // Ограничиваем поворот так, чтобы выбранная вершина всегда оставалась самой нижней.
        for (size_t i = 0; i < b.vertices.size(); ++i) {
            if (i == selected) continue;
            const Vector3 delta = peak.rotate(b.vertices[i] - tip);
            if (delta.z <= 0) throw std::runtime_error("ambiguous support vertex at sweep centre");
            sweep = std::min(sweep, .7 * std::atan2(double(delta.z), std::abs(double(delta.y))));
        }
        sweep = std::min(sweep, .3 * room / radius);
        if (room < .03 || radius * (1 - std::cos(sweep)) < 3 * kContactDistance)
            throw std::runtime_error("insufficient face clearance for verified rotation");
    }
};

enum class Path { Translate, Slide, Rotate };
struct Scene { const char* name; Path path; double minimum; bool atEdge; };

double amplitude(const Geometry& g, Path path) {
    return path == Path::Rotate ? g.radius * (1 - std::cos(g.sweep)) : .015;
}

double gapAt(const Geometry& g, const Scene& scene, double t) {
    if (scene.path == Path::Rotate)
        return scene.minimum + g.radius * (1 - std::cos((2*t - 1) * g.sweep));
    const double ramp = scene.path == Path::Slide ? std::max({0.0, 1 - 4*t, 4*t - 3}) : std::abs(2*t - 1);
    return scene.minimum + amplitude(g, scene.path) * ramp;
}

// Первое пересечение уровня на убывающей ветви; вторая ветвь зеркальна по времени.
double firstLevel(const Geometry& g, const Scene& scene, double level) {
    if (level < scene.minimum) return absent;
    if (level >= scene.minimum + amplitude(g, scene.path)) return 0;
    if (scene.path == Path::Rotate)
        return .5 - std::acos(1 - (level - scene.minimum) / g.radius) / (2*g.sweep);
    return (1 - (level - scene.minimum) / amplitude(g, scene.path)) / (scene.path == Path::Slide ? 4 : 2);
}

struct Poses { Transform a, b; };

Poses posesAt(const Geometry& g, const Scene& scene, double time, bool movingFrame) {
    Vector3 spot = scene.atEdge ? g.edge : g.centre;
    const bool slide = scene.path == Path::Slide;
    const double across = std::clamp(2 * (time - .25), 0.0, 1.0);
    if (slide) spot += Real(.15 * across) * (g.centre - g.edge);
    const double theta = scene.path == Path::Rotate ? (2*time - 1) * g.sweep : 0;
    const Quaternion initial = turnX(g.peakAngle);
    const auto rotation = turnX(g.peakAngle + theta);
    Vector3 position = spot - initial.rotate(g.tip);
    position.z += Real(scene.path == Path::Rotate ? scene.minimum : gapAt(g, scene, time));
    Transform base{};
    if (movingFrame) {
        base.position = {Real(1.2*time), Real(-.4*time), Real(.65*time)};
        base.orientation = Quaternion::fromRotationVector({Real(.3*time),Real(-.9*time),Real(.2*time)}).unit();
    }
    return {base, {base.point(position), (base.orientation * rotation).unit()}};
}

double speedBound(const Geometry& g, const Scene& scene) {
    if (scene.path == Path::Rotate) return 2*g.sweep*g.radius;
    if (scene.path == Path::Slide) return std::max(4*amplitude(g, scene.path), .3*(g.centre-g.edge).length());
    return 2*amplitude(g, scene.path);
}

bool measure(const ConvexHull& a, const ConvexHull& b, const Geometry& g, const Scene& scene,
             bool movingFrame, bool swap, bool reverse) {
    const auto at = [&](double t) { return posesAt(g, scene, reverse ? duration-t : t, movingFrame); };
    const auto distance = [&](Real t) {
        const Poses p = at(t);
        const auto gp = swap ? closestPoints(b,p.b,a,p.a) : closestPoints(a,p.a,b,p.b);
        return gp.coresOverlap ? Real(0) : gp.distance;
    };
    int evaluations = 0;
    // Общий перенос и поворот не меняют расстояние пары. Граница относится к относительной траектории.
    const Real actual = timeOfImpact([&](Real t) { ++evaluations; return distance(t); },
                                     Real(speedBound(g, scene) * (1 + 1e-12)), Real(duration), kContactDistance);
    const double first = firstLevel(g, scene, 0), band = firstLevel(g, scene, kContactDistance);
    const double half = firstLevel(g, scene, kContactDistance / 2);
    const bool pending = actual == duration;
    const bool hitOk = std::isfinite(band) ? !pending && actual >= band-timeTolerance && actual <= half+timeTolerance : pending;
    bool statesOk = true;
    int touching = 0, inside = 0, separated = 0;
    for (int i = 0; i <= 64; ++i) {
        const double t = duration*i/64;
        const Poses p = at(t);
        const double d = swap ? findSeparatingAxis(b,p.b,a,p.a,0).separation : findSeparatingAxis(a,p.a,b,p.b,0).separation;
        const double expected = gapAt(g,scene,reverse ? duration-t : t);
        if (!std::isfinite(d)) statesOk = false;
        if (expected > gapTolerance) { ++separated; statesOk &= d > 0; }
        else if (expected < -gapTolerance) { ++inside; statesOk &= d < 0; }
        else { ++touching; statesOk &= std::abs(d) <= gapTolerance; }
    }
    // Для обратного запроса также печатаем момент выхода в исходной шкале времени.
    const auto toForward = [&](double t) { return reverse ? duration-t : t; };
    std::printf("%s,%d,%d,%d,%.12g,%.12g,%.12g,%.12g,%.12g,%d,%d,%d,%d,%d,%d\n",
                scene.name, int(movingFrame), int(swap), int(reverse), double(kContactDistance),
                toForward(first), toForward(band), toForward(half), pending ? absent : toForward(actual),
                evaluations, separated, touching, inside, int(hitOk), int(statesOk));
    return hitOk && statesOk;
}
}

int main() {
    const auto ma = hull_sweep::shapeA(), mb = hull_sweep::shapeB();
    const kp::ConvexHull a(ma.vertices,ma.faces), b(mb.vertices,mb.faces);
    const Geometry g(ma,mb);
    const double band = kp::kContactDistance;
    const Scene scenes[] = {
        {"linear_cross",Path::Translate,-.004,false}, {"edge_graze",Path::Translate,0,true},
        {"edge_slide_leave",Path::Slide,0,true}, {"linear_close_miss",Path::Translate,1.2*band,false},
        {"linear_band_only",Path::Translate,.25*band,false},
        {"rotation_cross",Path::Rotate,-amplitude(g,Path::Rotate)/4,false},
        {"rotation_graze",Path::Rotate,0,false}, {"rotation_close_miss",Path::Rotate,1.2*band,false},
        {"rotation_band_only",Path::Rotate,.25*band,false}
    };
    std::puts("scenario,both_moving,swap,reverse,band_m,physical_event_s,oracle_band_event_s,oracle_half_band_event_s,"
              "observed_band_event_s,evaluations,outside_samples,touch_samples,inside_samples,toi_ok,sampled_states_ok");
    int failed = 0;
    for (const Scene& scene : scenes)
        for (bool moving : {false,true})
            for (bool swap : {false,true})
                for (bool reverse : {false,true})
                    failed += !measure(a,b,g,scene,moving,swap,reverse);
    std::fprintf(stderr,"scenarios=9 rows=72 failed_rows=%d angular_half_sweep_rad=%.12g\n",failed,g.sweep);
    return failed ? 1 : 0;
}
