// Отсчёты сцены viewer --measured 1. stdout: каждое тело каждого кадра; stderr: окно [60,74] с.
#include "../../../examples/scenes/StandardScenes.h"
#include <collision/PairFinder.h>
#include <math/WorkerPool.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string_view>

namespace {
using namespace kp;
constexpr int hz = 60, total = 75 * hz, first = 60 * hz, last = 74 * hz;
constexpr double degrees = 180.0 / 3.14159265358979323846;
constexpr Real dt = Real(1) / hz;
constexpr int bodyCount = 4; // пол и три одинаковых составных ведра

struct Contacts {
    int geometry = 0, floor = 0, buckets = 0, cache = 0, positive = 0;
    double floorNormals = 0, bucketNormals = 0;
};

// Тот же геометрический счёт, что debugGeometry и Panel::sceneInfo, без построения рёбер для отрисовки.
int contacts(const World& world, std::array<Contacts, bodyCount>& byBody) {
    int totalPoints = 0;
    for (const auto& pair : PairFinder(world.bodies(), world.gravity).pairs(0)) {
        if (!pair.inContact()) continue;
        const int points = pair.manifold.count;
        totalPoints += points;
        for (int id : {pair.a, pair.b}) {
            auto& c = byBody[size_t(id)];
            c.geometry += points;
            if (pair.a == 0 || pair.b == 0) c.floor += points;
            else c.buckets += points;
        }
    }
    // Это нормальные силы последнего решённого подшага. Сумма модулей не равна силе вдоль y.
    for (const auto& [pair, force] : StepObserver::cachedContactForces(world)) {
        for (int id : {pair.a, pair.b}) {
            auto& c = byBody[size_t(id)];
            ++c.cache;
            if (force.normal > 0) ++c.positive;
            if (pair.a == 0 || pair.b == 0) c.floorNormals += force.normal;
            else c.bucketNormals += force.normal;
        }
    }
    return totalPoints;
}

struct Pose {
    Vector3 p;
    Quaternion q;
};

double turnDegrees(Quaternion current, Quaternion reference) {
    const Quaternion q = current * reference.conjugate();
    return 2 * degrees * std::atan2(std::hypot(double(q.x), double(q.y), double(q.z)), std::abs(double(q.w)));
}

struct LateWindow {
    Pose reference{};
    int count = 0, speedViolations = 0;
    double displacement = 0, rotation = 0, frameMove = 0, frameTurn = 0, tilt = 0, pointSpeed = 0;

    void record(int frame, Pose now, double deltaMm, double deltaDeg, double tiltDeg, double speed) {
        if (frame == first) reference = now;
        if (frame < first || frame > last) return;
        ++count;
        displacement = std::max(displacement, 1000.0 * double((now.p - reference.p).length()));
        rotation = std::max(rotation, turnDegrees(now.q, reference.q));
        tilt = std::max(tilt, tiltDeg);
        pointSpeed = std::max(pointSpeed, speed);
        speedViolations += speed > 0.02; // порог скорости из bucketsVerdict
        if (frame == first) return; // в дельту окна входят только соседние отсчёты из этого же окна
        frameMove = std::max(frameMove, deltaMm);
        frameTurn = std::max(frameTurn, deltaDeg);
    }
};

bool run(const StandardScene& scene, bool coupled) {
    World world = scene.build();
    if (world.bodies().size() != bodyCount || !world.bodies().front().isStatic()) return false;
    world.realtime = true;
    world.soft.coupledFriction = coupled;
    world.sleeping = world.shockPropagation = false;
    world.sleepSpeed = Real(0.05);
    world.sleepTime = 2;
    world.maxSegments = 8;
    world.soft.rollingResistance = 0;
    std::array<Vector3, bodyCount> upInBody{};
    std::array<Pose, bodyCount> previous{};
    std::array<LateWindow, bodyCount> windows{};
    for (int i = 0; i < bodyCount; ++i) {
        Body& b = world.body(BodyId(i));
        b.linearDamping = b.angularDamping = 0;
        previous[i] = {b.position, b.orientation};
        // Главные оси инерции составного тела могут отличаться от осей исходной геометрии.
        upInBody[i] = b.orientation.rotateInverse({0, 1, 0});
        if (i > 0 && (b.isStatic() || !b.parts)) return false;
    }
    const char* mode = coupled ? "coupled" : "stock";
    std::fprintf(stderr, "mode=%s threads=%d scalar_bytes=%zu substeps=%d contact_updates=%d sleep=0 drag=0\n",
                 mode, WorkerPool::shared().threads(), sizeof(Real), world.soft.substeps, world.soft.contactUpdates);
    std::puts("mode,frame,time_s,body,com_x_m,com_y_m,com_z_m,qw,qx,qy,qz,vx_m_s,vy_m_s,vz_m_s,"
              "wx_rad_s,wy_rad_s,wz_rad_s,tilt_deg,frame_move_mm,frame_turn_deg,point_speed_bound_m_s,"
              "viewer_points_world,viewer_points_body,viewer_points_floor_body,viewer_points_bucket_body,"
              "cache_rows_body,cache_positive_rows_body,cache_floor_normal_sum_N,cache_bucket_normal_sum_N");
    for (int frame = 0; frame <= total; ++frame) {
        if (frame > 0) {
            if (scene.control) scene.control(world, double(dt) * (frame - 1));
            world.step(dt);
            if (world.report().stalled || world.report().unfinished != 0) {
                std::fprintf(stderr, "complete=0 failed_frame=%d\n", frame);
                return false;
            }
        }
        std::array<Contacts, bodyCount> byBody{};
        const int points = contacts(world, byBody);
        for (int i = 1; i < bodyCount; ++i) {
            const auto& b = world.bodies()[size_t(i)];
            const auto p = b.position, v = b.linearVelocity(), w = b.angularVelocity();
            const auto q = b.orientation;
            const auto up = q.rotate(upInBody[i]);
            const double tilt = degrees * std::atan2(std::hypot(double(up.x), double(up.z)), double(up.y));
            const double move = 1000 * double((p - previous[i].p).length());
            const double turn = turnDegrees(q, previous[i].q);
            const double speed = double(v.length()) + double(w.length()) * double(b.boundingRadius());
            if (!std::isfinite(tilt + move + turn + speed)) {
                std::fprintf(stderr, "complete=0 nonfinite_frame=%d body=%d\n", frame, i);
                return false;
            }
            const auto& c = byBody[i];
            std::printf("%s,%d,%.12g,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,"
                        "%.17g,%.17g,%.17g,%.12g,%.12g,%.12g,%.12g,%d,%d,%d,%d,%d,%d,%.12g,%.12g\n",
                        mode, frame, double(dt) * frame, i, double(p.x), double(p.y), double(p.z), double(q.w),
                        double(q.x), double(q.y), double(q.z), double(v.x), double(v.y), double(v.z),
                        double(w.x), double(w.y), double(w.z), tilt, move, turn, speed, points, c.geometry,
                        c.floor, c.buckets, c.cache, c.positive, c.floorNormals, c.bucketNormals);
            windows[i].record(frame, {p, q}, move, turn, tilt, speed);
            previous[i] = {p, q};
        }
    }
    std::fprintf(stderr, "body,samples,max_com_from_60s_mm,max_turn_from_60s_deg,max_frame_move_mm,max_frame_turn_deg,"
                         "max_tilt_deg,max_point_speed_bound_m_s,frames_over_0p02_m_s,within_1mm,within_1deg\n");
    for (int i = 1; i < bodyCount; ++i) {
        const auto& s = windows[i];
        std::fprintf(stderr, "%d,%d,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%d,%d,%d\n", i, s.count, s.displacement,
                     s.rotation, s.frameMove, s.frameTurn, s.tilt, s.pointSpeed, s.speedViolations,
                     int(s.displacement <= 1), int(s.rotation <= 1));
    }
    std::fputs("complete=1 frames=4500\n", stderr);
    return true;
}
}

int main(int argc, char** argv) {
    const std::string_view mode = argc == 2 ? argv[1] : "stock";
    if (argc > 2 || (mode != "stock" && mode != "coupled")) return 2;
    const char* threads = std::getenv("KP_THREADS");
    kp::WorkerPool::shared().resize(threads ? std::max(1, std::atoi(threads)) : 1);
    for (const auto& scene : standardScenes())
        if (std::string_view(scene.key) == "buckets") return run(scene, mode == "coupled") ? 0 : 1;
    return 2;
}
