#pragma once
#include <algorithm>

// Шаг мира фиксирован. Частота экрана определяет только число перерисовок.
class PlaybackClock {
public:
    PlaybackClock(double now, double step) : previous_(now), step_(step) {}

    int steps(double now, bool paused, bool singleStep, bool recording) {
        const double elapsed = now - previous_;
        previous_ = now;
        if (paused || recording) {
            pending_ = 0;
            return paused ? int(singleStep) : 1;
        }
        // После остановки отладчиком или сворачивания окно остаётся отзывчивым.
        pending_ = std::min(pending_ + elapsed, 4 * step_);
        const int count = int(pending_ / step_);
        pending_ -= count * step_;
        return count;
    }
private:
    double previous_, step_, pending_ = 0;
};
