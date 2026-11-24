// Replays wall-clock samples against the clock used by the interactive viewer.
#include "../../viewer/PlaybackClock.h"
#include <cmath>
#include <cstdio>

int main() {
    constexpr double step = 1.0 / 60;
    std::puts("display_hz,real_seconds,physics_frames,physics_seconds,pass");
    bool pass = true;
    for (const int hz : {30, 60, 120, 144, 165, 240}) {
        PlaybackClock clock(0, step);
        int frames = 0;
        for (int i = 1; i <= 10 * hz; ++i)
            frames += clock.steps(double(i) / hz, false, false, false);
        const bool correct = frames <= 600 && frames >= 599;
        pass &= correct;
        std::printf("%d,10,%d,%.9g,%d\n", hz, frames, frames * step, int(correct));
    }
    PlaybackClock clock(0, step);
    const bool pause = clock.steps(5, true, false, false) == 0;
    const bool single = clock.steps(8, true, true, false) == 1;
    const bool resume = clock.steps(8.001, false, false, false) == 0;
    const bool bounded = clock.steps(12, false, false, false) == 4;
    const bool exportFrame = clock.steps(12.001, false, false, true) == 1;
    std::fprintf(stderr, "pause=%d single_step=%d resume=%d stall_limit=%d export_one=%d\n",
                 pause, single, resume, bounded, exportFrame);
    return pass && pause && single && resume && bounded && exportFrame ? 0 : 1;
}
