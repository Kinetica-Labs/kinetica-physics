// Первые три проверки на небольшом мире. Затем 75 с штатных buckets, вывод [60,74] с.
// Интервал считается изменившимся, если точка или направленный отрезок потеряли соседа в радиусе 1 мм.
#include "../../viewer/ContactOverlay.h"
#include "OverlayComparison.h"
#include "../../examples/scenes/StandardScenes.h"
#include <math/WorkerPool.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string_view>

namespace {
using namespace kp;
using namespace overlay_test;
constexpr double degrees = 180.0 / 3.14159265358979323846;

// Полный прежний путь viewer. Из него выбираем ровно те же типы, которые теперь отдаёт overlay.
DebugGeometry original(const World& world) {
    DebugGeometry out;
    const auto debug = debugGeometry(world);
    for (const auto& p : debug.points) if (p.kind == DebugKind::ContactPoint) out.points.push_back(p);
    for (const auto& l : debug.lines) if (l.kind == DebugKind::ContactNormal) out.lines.push_back(l);
    return out;
}

World restingBox() {
    World world;
    world.addBody(Body::box({2, Real(0.25), 2}, 0, {0, Real(-0.25), 0}));
    world.addBody(Body::box({Real(0.25), Real(0.125), Real(0.375)}, 1, {0, Real(0.125), 0}));
    return world;
}
bool check(const char* label, bool pass) {
    std::fprintf(stderr, "check=%s pass=%d\n", label, int(pass));
    return pass;
}

bool tinyMovement(bool rotate) {
    World world = restingBox();
    ContactOverlay overlay;
    const DebugGeometry initial = overlay.sample(world);
    bool firstHeld = false;
    int refresh = 0;
    // Каждый вход отмеряется от исходной позы. Даже очень малые повороты не теряются при сложении.
    constexpr Real increment = Real(1e-7) / 16; // м пути точки поверхности
    for (int k = 1; k <= 32; ++k) {
        const Real offset = increment * Real(k);
        auto& box = world.body(BodyId(1));
        if (rotate) box.orientation = Quaternion::fromRotationVector({0, offset / box.boundingRadius(), 0});
        else box.position.x = offset;
        const auto& markers = overlay.sample(world);
        if (k == 1) firstHeld = exact(markers, initial);
        if (!exact(markers, initial)) {
            if (exact(markers, original(world))) refresh = k;
            break;
        }
    }
    std::fprintf(stderr, "motion=%s first_refresh_step=%d increment=%.12g\n",
                 rotate ? "rotation_surface_m" : "translation_m", refresh, double(increment));
    return check(rotate ? "accumulated_rotation" : "accumulated_translation",
                 !initial.points.empty() && !initial.lines.empty() && firstHeld && refresh >= 16 && refresh <= 17);
}

bool modes() {
    World world = restingBox();
    ContactOverlay overlay;
    const auto initial = overlay.sample(world);
    bool ok = check("first_sample", !initial.points.empty() && !initial.lines.empty() && exact(initial, original(world)));
    auto& b = world.body(BodyId(1));
    b.position.x = 2 * kEpsilon;
    const auto full = original(world);
    ok &= check("raw_tiny_translation", !exact(initial, full) && exact(overlay.sample(world, true), full));
    ok &= check("raw_does_not_replace_held_patch", exact(overlay.sample(world), initial));
    overlay.clear();
    b.position.x = 0;
    const auto before = overlay.sample(world);
    b.position.x = 2 * kEpsilon;
    const auto held = overlay.sample(world);
    overlay.clear();
    ok &= check("clear_tiny_translation", exact(before, initial) && exact(held, initial) && exact(overlay.sample(world), full));
    b.position.x = Real(0.1);
    const auto translated = overlay.sample(world);
    ok &= check("translation", visiblyDifferent(translated.points, initial.points) && exact(translated, original(world)));
    b.orientation = Quaternion::fromRotationVector({0, Real(0.25), 0});
    const auto turned = overlay.sample(world);
    ok &= check("rotation", visiblyDifferent(turned.lines, translated.lines) && exact(turned, original(world)));
    b.position.y += 1;
    const auto& separated = overlay.sample(world);
    ok &= check("contact_loss", separated.points.empty() && separated.lines.empty() && exact(separated, original(world)));
    return ok;
}

DebugGeometry sideOf(const DebugGeometry& geometry, bool left) {
    DebugGeometry result;
    for (const auto& p : geometry.points) if ((p.position.x < Real(.6)) == left) result.points.push_back(p);
    for (const auto& l : geometry.lines) if ((l.from.x < Real(.6)) == left) result.lines.push_back(l);
    return result;
}

bool independentPairs() {
    World world = restingBox();
    world.addBody(Body::box({Real(.25), Real(.125), Real(.375)}, 1, {Real(1.2), Real(.125), 0}));
    ContactOverlay overlay;
    const auto initial = overlay.sample(world);
    world.body(BodyId{1}).position.x += Real(5e-8);
    world.body(BodyId{2}).position.x += Real(.02);
    const auto& shown = overlay.sample(world);
    const auto raw = original(world);
    return check("independent_pairs", !sideOf(initial, true).points.empty() &&
                 !exact(sideOf(initial, true), sideOf(raw, true)) &&
                 exact(sideOf(shown, true), sideOf(initial, true)) &&
                 !exact(sideOf(shown, false), sideOf(initial, false)) && exact(sideOf(shown, false), sideOf(raw, false)));
}

bool grazingRelease() {
    World world = restingBox();
    auto& b = world.body(BodyId{1});
    b.position.y += kContactDistance - Real(1e-8);
    ContactOverlay overlay;
    const bool held = !overlay.sample(world).points.empty();
    b.position.y += Real(2e-8); // всего 20 нм, но целая пара вышла из полосы касания
    const bool lost = overlay.sample(world).points.empty() && original(world).points.empty();
    b.position.y -= Real(2e-8);
    return check("grazing_release_and_return", held && lost && !overlay.sample(world).points.empty());
}

struct Pose { Vector3 p; Quaternion q; };
struct Motion { double metres = 0, degreesTurned = 0; };
using Poses = std::array<Pose, 3>;

Poses poses(const World& world) {
    Poses result{};
    for (int i = 0; i < 3; ++i) {
        const auto& body = world.bodies()[size_t(i + 1)];
        result[i] = {body.position, body.orientation};
    }
    return result;
}
Motion motion(const Pose& now, const Pose& before) {
    const Quaternion q = now.q * before.q.conjugate();
    return {double((now.p - before.p).length()),
            2 * degrees * std::atan2(std::hypot(double(q.x), double(q.y), double(q.z)), std::abs(double(q.w)))};
}
Motion motion(const Poses& now, const Poses& before) {
    Motion out;
    for (int i = 0; i < 3; ++i) {
        const Motion m = motion(now[i], before[i]);
        out.metres = std::max(out.metres, m.metres);
        out.degreesTurned = std::max(out.degreesTurned, m.degreesTurned);
    }
    return out;
}

bool moving(Motion m) { return m.metres > 1e-6 || m.degreesTurned > 1e-4; }

// Оракул независимости: отрисовать каждую из шести пар отдельно, в тех же позах.
// В движущейся паре все отметки обязаны совпасть со свежим поиском. Отметки
// остальных пар вместе с ними обязаны дать в точности то, что показал общий мир.
struct IsolatedPair {
    World world;
    ContactOverlay overlay;
};
struct PairOracle {
    std::array<IsolatedPair, 6> pairs;
    int movingContacts = 0, movingFailures = 0, sceneFailures = 0;

    void observe(const World& scene, const Poses& now, const Poses& before, const DebugGeometry& shown) {
        std::array<bool, 4> moved{};
        for (size_t i = 0; i < now.size(); ++i) moved[i+1] = moving(motion(now[i], before[i]));
        DebugGeometry together;
        size_t k = 0;
        for (int a = 0; a < 4; ++a) for (int b = a+1; b < 4; ++b) {
            auto& pair = pairs[k++];
            if (pair.world.bodies().empty()) {
                pair.world.addBody(Body{});
                pair.world.addBody(Body{});
            }
            pair.world.body(BodyId{0}) = scene.body(BodyId{a});
            pair.world.body(BodyId{1}) = scene.body(BodyId{b});
            const auto& markers = pair.overlay.sample(pair.world);
            if (moved[size_t(a)] || moved[size_t(b)]) {
                const auto fresh = original(pair.world);
                movingContacts += !fresh.points.empty();
                movingFailures += !exact(markers, fresh);
            }
            together.lines.insert(together.lines.end(), markers.lines.begin(), markers.lines.end());
            together.points.insert(together.points.end(), markers.points.begin(), markers.points.end());
        }
        sceneFailures += !exact(together, shown);
    }
};

bool buckets(const StandardScene& scene) {
    constexpr int hz = 60, first = 60 * hz, last = 74 * hz;
    constexpr Real h = Real(1) / hz;
    World world = scene.build();
    if (world.bodies().size() != 4) return false;
    world.realtime = true;
    world.sleeping = world.shockPropagation = false;
    world.maxSegments = 8;
    world.soft.rollingResistance = 0;
    for (size_t i = 0; i < world.bodies().size(); ++i) {
        auto& body = world.body(BodyId(int(i)));
        body.linearDamping = body.angularDamping = 0;
    }
    ContactOverlay overlay, rawOverlay;
    DebugGeometry previousRaw, previousStable;
    Poses previousPose = poses(world), startPose{};
    int comparisons = 0, rawPoints = 0, rawLines = 0, stablePoints = 0, stableLines = 0;
    int movingFrames = 0, fullRawDifferences = 0;
    PairOracle oracle;
    Motion peak{};
    std::puts("frame,time_s,raw_points,overlay_points,raw_points_changed,raw_normals_changed,overlay_points_changed,"
              "overlay_normals_changed,step_move_m,step_turn_deg,move_from_60s_m,turn_from_60s_deg");
    for (int frame = 0; frame <= 75 * hz; ++frame) {
        if (frame) {
            if (scene.control) scene.control(world, double(h) * (frame - 1));
            world.step(h);
            if (world.report().stalled || world.report().unfinished != 0) return false;
        }
        const Poses now = poses(world);
        const Motion step = motion(now, previousPose);
        const auto& current = overlay.sample(world);
        const auto& raw = rawOverlay.sample(world, true);
        oracle.observe(world, now, previousPose, current);
        if (moving(step)) {
            ++movingFrames;
            fullRawDifferences += !exact(current, raw);
        }
        if (frame == first) startPose = now;
        if (frame >= first && frame <= last) {
            const Motion change = motion(now, startPose);
            const bool compare = frame > first;
            const bool rp = compare && visiblyDifferent(raw.points, previousRaw.points);
            const bool rl = compare && visiblyDifferent(raw.lines, previousRaw.lines);
            const bool cp = compare && visiblyDifferent(current.points, previousStable.points);
            const bool cl = compare && visiblyDifferent(current.lines, previousStable.lines);
            rawPoints += rp; rawLines += rl; stablePoints += cp; stableLines += cl; comparisons += compare;
            peak.metres = std::max(peak.metres, change.metres);
            peak.degreesTurned = std::max(peak.degreesTurned, change.degreesTurned);
            std::printf("%d,%.12g,%zu,%zu,%d,%d,%d,%d,%.12g,%.12g,%.12g,%.12g\n", frame, double(h) * frame,
                        raw.points.size(), current.points.size(), int(rp), int(rl), int(cp), int(cl),
                        step.metres, step.degreesTurned, change.metres, change.degreesTurned);
            previousRaw = raw;
            previousStable = current;
        }
        previousPose = now;
    }
    std::fprintf(stderr, "late_intervals=%d raw_point_changes=%d raw_normal_changes=%d overlay_point_changes=%d "
                 "overlay_normal_changes=%d max_real_from_60s_m=%.12g max_real_from_60s_deg=%.12g "
                 "moving_frames=%d full_raw_differences=%d moving_contacts=%d moving_pair_failures=%d "
                 "isolated_scene_failures=%d complete=1\n", comparisons, rawPoints, rawLines,
                 stablePoints, stableLines, peak.metres, peak.degreesTurned, movingFrames, fullRawDifferences,
                 oracle.movingContacts, oracle.movingFailures, oracle.sceneFailures);
    return check("rest_overlay", comparisons == last - first &&
                 stablePoints == 0 && stableLines == 0 && movingFrames > 0 && oracle.movingContacts > 0 &&
                 oracle.movingFailures == 0 && oracle.sceneFailures == 0);
}
}

int main() {
    const char* threads = std::getenv("KP_THREADS");
    kp::WorkerPool::shared().resize(threads ? std::max(1, std::atoi(threads)) : 1);
    bool ok = modes();
    ok &= tinyMovement(false);
    ok &= tinyMovement(true);
    ok &= independentPairs();
    ok &= grazingRelease();
    for (const auto& scene : standardScenes())
        if (std::string_view(scene.key) == "buckets") return (buckets(scene) && ok) ? 0 : 1;
    return 2;
}
