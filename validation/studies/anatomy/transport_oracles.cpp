// Один и тот же прибор линкуется с исходным и опытным ядром. Все наблюдения — после публичного шага.
// Контракт: максимум ошибки центра и проникновение <= 0,5 мм, максимум ошибки скорости <= 1 мкм/с.
// В свободном полёте y=y0+v0*t+g*t²/2. При центральном упругом ударе y=R+u*|t-t_hit|.
#include <math/WorkerPool.h>
#include <world/World.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace {
constexpr double radius = 0.125; // м, верх плиты находится в y=0
constexpr double speed = 3.0;    // м/с, выше штатного порога отскока
constexpr double positionTolerance = 0.0005, velocityTolerance = 1e-6;
constexpr double missing = std::numeric_limits<double>::quiet_NaN();

struct Scene {
    const char* name;
    double initialY, initialVelocity, gravity, hitTime;

    bool hasPlate() const { return hitTime >= 0; }
    double y(double t) const {
        return hasPlate() ? radius + speed * std::abs(t - hitTime)
                          : initialY + initialVelocity * t + 0.5 * gravity * t * t;
    }
    double velocity(double t) const {
        if (hasPlate()) return t < hitTime ? -speed : speed;
        return initialVelocity + gravity * t;
    }
};

// Работаем в одной вертикальной плоскости симметрии. Поперечный уход также входит в ошибку.
double distanceFromAxisTarget(const kp::Vector3& actual, double targetY) {
    return std::hypot(double(actual.x), double(actual.y) - targetY, double(actual.z));
}

bool measure(const Scene& scene, int hz) {
    kp::World world;
    world.realtime = true;
    world.soft.substeps = world.soft.contactUpdates = 1;
    world.soft.coupledFriction = true;
    world.sleeping = false;
    world.gravity = {0, kp::Real(scene.gravity), 0};
    if (scene.hasPlate()) {
        kp::Body plate = kp::Body::box({2, kp::Real(0.25), 2}, 0, {0, kp::Real(-0.25), 0});
        plate.friction = 0;
        plate.restitution = 1;
        world.addBody(plate);
    }
    kp::Body sphere = kp::Body::sphere(kp::Real(radius), 1000, {0, kp::Real(scene.initialY), 0});
    sphere.friction = 0;
    sphere.restitution = 1;
    sphere.linearMomentum.y = kp::Real(scene.initialVelocity) / sphere.mass.inverseMass;
    const auto id = world.addBody(sphere);

    const int steps = 8 * hz / 4; // 0,25 с в каждой сцене при обеих частотах
    const double h = 1.0 / (8 * hz);
    double positionError = 0, velocityError = 0, penetration = 0;
    double y = sphere.position.y, vy = sphere.linearVelocity().y;
    double before = scene.hasPlate() ? vy : missing, after = missing;
    int completed = 0, firstViolation = 0;
    for (int i = 1; i <= steps; ++i) {
        world.step(kp::Real(h));
        const auto& ball = world.body(id);
        const auto velocity = ball.linearVelocity();
        const double t = i * h;
        const double dp = distanceFromAxisTarget(ball.position, scene.y(t));
        const double dv = distanceFromAxisTarget(velocity, scene.velocity(t));
        if (world.report().stalled || world.report().unfinished != 0 || !std::isfinite(dp) || !std::isfinite(dv)) break;
        completed = i;
        y = ball.position.y;
        vy = velocity.y;
        positionError = std::max(positionError, dp);
        velocityError = std::max(velocityError, dv);
        if (scene.hasPlate()) {
            penetration = std::max(penetration, radius - y);
            if (t < scene.hitTime) before = vy;
            if (t > scene.hitTime && std::isnan(after)) after = vy;
        }
        if (!firstViolation && (dp > positionTolerance || dv > velocityTolerance || penetration > positionTolerance))
            firstViolation = i;
    }
    const bool complete = completed == steps;
    const double t = completed * h;
    std::printf("%s,%d,%zu,%d,%.12g,%d,%d,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                "%.12g,%.12g,%.12g,%.12g,%.12g,%d,%d,%d\n",
                scene.name, hz, sizeof(kp::Real), 8, h, steps, completed, t, scene.initialY, scene.initialVelocity,
                scene.hasPlate() ? scene.hitTime : missing, y, scene.y(t), 1000 * positionError, velocityError,
                before, after, vy, scene.velocity(t), scene.hasPlate() ? 1000 * penetration : missing,
                firstViolation, int(complete), int(complete && firstViolation == 0));
    return complete;
}
}

int main() {
    const char* threads = std::getenv("KP_THREADS");
    kp::WorkerPool::shared().resize(threads ? std::max(1, std::atoi(threads)) : 1);
    // (8*60)/36 = 13+1/3, (8*240)/36 = 53+1/3. Обе сетки видят один и тот же удар внутри шага.
    constexpr double inside = 1.0 / 36;
    const Scene scenes[] = {{"free_motion", 1, speed, 0, -1}, {"free_fall", 1, 0, -9.81, -1},
                            {"impact_inside", radius + speed * inside, -speed, 0, inside},
                            {"impact_at_start", radius, -speed, 0, 0}};
    std::puts("scene,frame_hz,scalar_bytes,public_steps_per_frame,h_s,requested_steps,completed_steps,time_s,"
              "initial_y_m,initial_vy_m_s,oracle_hit_s,y_m,oracle_y_m,max_position_error_mm,max_velocity_error_m_s,"
              "last_pre_hit_vy_m_s,first_post_hit_vy_m_s,vy_m_s,oracle_vy_m_s,max_sampled_penetration_mm,"
              "first_violation_step,complete,pass");
    bool completed = true;
    for (int hz : {60, 240})
        for (const auto& scene : scenes)
            if (!measure(scene, hz)) completed = false;
    return completed ? 0 : 1; // Физический критерий каждой завершившейся строки — отдельное поле pass.
}
