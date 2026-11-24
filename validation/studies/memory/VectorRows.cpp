// Цена сборки пяти строк обычного ползуна. Все три варианта передают одни данные одному потребителю.
#include <world/World.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <limits>
#include <vector>

double readRows(const kp::Bond* rows, std::size_t size);

int main() {
    kp::World world;
    const auto base = world.addBody(kp::Body{});
    const auto body = world.addBody(kp::Body::box({1,1,1}, 1));
    const auto& slider = kp::addSlider(world, base, body, {}, {1,0,0});
    const auto source = slider.bonds(world.bodies(), world.bodies());
    constexpr std::size_t count = 5;
    if (source.size() != count) return 2;
    std::vector<kp::Bond> reused;
    reused.reserve(count);
    std::array<kp::Bond, count> fixed;
    constexpr int samples = 150000;
    const char* names[] = {"new-vector", "reuse-vector", "fixed-array"};
    std::puts("block,slot,storage,lists,ns_per_list,checksum");
    double answer = std::numeric_limits<double>::quiet_NaN();
    for (int block = -1; block < 15; ++block) {
        for (int slot = 0; slot < 3; ++slot) {
            const int method = (block + 1 + slot) % 3;
            double sum = 0;
            const auto start = std::chrono::steady_clock::now();
            for (int i = 0; i < samples; ++i) {
                if (method == 0) {
                    std::vector<kp::Bond> temporary;
                    for (const auto& row : source) temporary.push_back(row);
                    sum += readRows(temporary.data(), temporary.size());
                } else if (method == 1) {
                    reused.clear();
                    for (const auto& row : source) reused.push_back(row);
                    sum += readRows(reused.data(), reused.size());
                } else {
                    std::copy(source.begin(), source.end(), fixed.begin());
                    sum += readRows(fixed.data(), fixed.size());
                }
            }
            const auto elapsed = std::chrono::steady_clock::now() - start;
            if (block < 0) { answer = sum; continue; }
            if (sum != answer) return 1;
            const double ns = std::chrono::duration<double, std::nano>(elapsed).count() / samples;
            std::printf("%d,%d,%s,%d,%.9g,%.17g\n", block, slot, names[method], samples, ns, sum);
        }
    }
}
