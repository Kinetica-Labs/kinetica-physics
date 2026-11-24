#include "Viewer.h"
#include "FloatingBox.h"
#include "GearTrain.h"
#include <cmath>
#include <cstdio>

// Графику показывает тот же собранный на стенде viewer. Здесь проверяется шаг симуляции окна.
void drawWaterSurface(const kp::World&) {}
void drawGearMarks(const kp::World&) {}

int main() {
    if (sceneIndex("rail") != sceneIndex("34") || sceneIndex("floating-box") != sceneIndex("39") ||
        sceneIndex("gear-train") != sceneIndex("40") || sceneIndex("hull-events") != sceneIndex("37") ||
        sceneIndex("cylinder-rim") != sceneIndex("38") || sceneIndex("missing-key") != scenes().size()) return 1;
    for (bool realtime : {false, true}) {
        Viewer view;
        view.realtime = realtime;
        view.load(sceneIndex("floating-box"));
        view.measure();
        for (int i = 0; i < 120; ++i) view.step();
        std::printf("water realtime=%d max_draft_error_cm=%.9g lag_s=%.9g\n", realtime, view.worstError, view.lag);
        if (!std::isfinite(view.worstError) || view.worstError > 0.1 || view.lag != 0) return 2;
        auto& box = view.world.body(floating_box::boxId);
        kp::applyImpulse(box, {0.1, 0.12, 0.03}, box.position + kp::Vector3{0.05, 0, 0});
        for (int i = 0; i < 60; ++i) view.step();
        const double tilt = (box.orientation.rotate({0,1,0}) - kp::Vector3{0,1,0}).length();
        std::printf("guided lateral_m=%.9g tilt=%.9g\n", std::hypot(box.position.x,box.position.z), tilt);
        if (!std::isfinite(tilt) || tilt > 1e-5 || std::hypot(box.position.x,box.position.z)>1e-5) return 3;
        view.load(sceneIndex("gear-train"));
        for (int i = 0; i < 120; ++i) view.step();
        std::printf("gears realtime=%d max_speed_error_rad_s=%.9g lag_s=%.9g\n", realtime, view.worstError, view.lag);
        if (!std::isfinite(view.worstError) || view.worstError>1e-7 || view.lag != 0) return 4;
    }
    Viewer rail;
    rail.load(sceneIndex("rail"));
    rail.measure();
    for (int i = 0; i < 420; ++i) rail.step();
    std::printf("rail frame=%d judged=%d verdict=%s\n", rail.frame, rail.judged, rail.verdict.c_str());
    return rail.judged && rail.verdict.empty() ? 0 : 5;
}
