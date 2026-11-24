// Each executable drives its usual grip adapter through the same force grid.
#define main originalGripMain
#if defined(KP_GRID_BOX3D)
#include "../../engines/grip_box3d.cpp"
#elif defined(KP_GRID_RP3D)
#include "../../engines/grip_rp3d.cpp"
#else
#include "../GripLimit.cpp"
#endif
#undef main

namespace {
grip::Result sample(grip::Load load, int hz) {
#if defined(KP_GRID_BOX3D) || defined(KP_GRID_RP3D)
    Rig rig(load, true);
#else
    Rig rig(1, load, true);
    rig.world().soft.coupledFriction = true;
#endif
    return grip::measure(rig, load, hz);
}
}

int main() {
#if !defined(KP_GRID_BOX3D) && !defined(KP_GRID_RP3D)
    kp::WorkerPool::shared().resize(1);
#endif
    std::puts("hz,force_N,oracle_mm,terminal_mm,max_path_error_mm,left_error_N,right_error_N,complete");
    for (int hz : {60, 120, 240}) {
        for (double magnitude : {0., 8., 9., 9.5, 9.8, 9.9, 9.95, 9.975, 10., 10.025, 10.05, 10.1, 10.2, 10.5, 11., 12.}) {
            for (int sign : {1, -1}) {
                if (magnitude == 0 && sign < 0) continue;
                const grip::Load load{0.5, 10, sign * magnitude};
                const auto r = sample(load, hz);
                std::printf("%d,%.4f,%.12g,%.12g,%.12g,%.12g,%.12g,%d\n", hz, load.down,
                            1000 * grip::acceleration(load) * .25 * .25 / 2, 1000 * r.drop,
                            1000 * r.maxPathError, r.maxLeftError, r.maxRightError, int(r.complete));
                if (!r.complete) return 1;
            }
        }
    }
}
