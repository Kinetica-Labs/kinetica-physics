#pragma once
#include <array>

namespace stress {
constexpr double halfSide = .125; // м
constexpr std::array<int,10> pyramidSides{16,14,13,12,10,8,6,5,3,1};

// Порядок слоя, ряда и столбца задаёт одни и те же номера тел в окне и в приборах.
template<class Add>
void forEachCube(bool rain, Add add) {
    const auto outward = [](double p) { return 4.0 * ((p > 0) - (p < 0)); };
    for (int layer = 0; layer < 10; ++layer) {
        const int side = rain ? 10 : pyramidSides[layer];
        const double pitch = rain ? .375 : 2*halfSide + 1./512;
        const double y = rain ? 1.125 + .375*layer : halfSide*(1 + 2*layer);
        const bool slides = !rain && layer % 2 == 0;
        for (int z = 0; z < side; ++z) for (int x = 0; x < side; ++x) {
            const double px = pitch*(x-(side-1)*.5), pz = pitch*(z-(side-1)*.5);
            add(px, y, pz, slides ? outward(px) : 0., slides ? outward(pz) : 0.);
        }
    }
}
}
