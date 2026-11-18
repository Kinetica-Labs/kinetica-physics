// Отпечаток состояния: хеш положений, ориентаций и импульсов всех тел после заданного числа
// кадров стандартных сцен и сцен с точным решением (сочленения), точным шагом и шагом с
// постоянной ценой кадра. Перестановка кода, которая не должна менять физику, обязана оставить
// каждый отпечаток прежним бит в бит.
//
//   study-state-hash [кадров на сцену, по умолчанию 60]

#include "../../examples/scenes/ExactScenes.h"
#include "../../examples/scenes/StandardScenes.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace kp;

namespace {

// FNV-1a по байтам чисел.
struct Hash {
    std::uint64_t value = 1469598103934665603ull;
    void add(Real x) {
        unsigned char bytes[sizeof(Real)];
        std::memcpy(bytes, &x, sizeof(Real));
        for (unsigned char byte : bytes) value = (value ^ byte) * 1099511628211ull;
    }
    void add(Vector3 v) { add(v.x), add(v.y), add(v.z); }
};

// control — управление сценой по времени (захват); maxSegments — предел отрезков точного шага.
std::uint64_t after(World (*build)(), void (*control)(World&, double), int frames, bool realtime, int maxSegments = 0) {
    World world = build();
    world.realtime = realtime;
    world.maxSegments = maxSegments;
    for (int frame = 0; frame < frames; ++frame) {
        if (control) control(world, frame / 60.0);
        world.step(Real(1) / 60);
    }
    Hash hash;
    for (const Body& body : world.bodies()) {
        hash.add(body.position), hash.add(body.linearMomentum), hash.add(body.angularMomentum);
        hash.add(body.orientation.w), hash.add(body.orientation.x), hash.add(body.orientation.y), hash.add(body.orientation.z);
    }
    return hash.value;
}

// step выровнен пробелами заранее: %-8s считал бы байты, а у кириллицы их по два на букву.
void print(const char* key, const char* step, std::uint64_t value) {
    std::printf("%-9s %s%016llx\n", key, step, static_cast<unsigned long long>(value));
    std::fflush(stdout);
}

} // namespace

int main(int argc, char** argv) {
    const int frames = argc > 1 ? std::atoi(argv[1]) : 60;
    for (const StandardScene& scene : standardScenes()) {
        // Стена точным шагом считается минутами: её отпечаток берётся только у быстрого шага.
        if (std::strcmp(scene.key, "wall") != 0) print(scene.key, "точный   ", after(scene.build, scene.control, frames, false));
        print(scene.key, "быстрый  ", after(scene.build, scene.control, frames, true));
    }
    // Сочленения точного шага (подбор SHAKE) есть только в сценах с точным решением. Предел в 64
    // отрезка — как у прибора study-exact-scenes; цепь 100:1 точным шагом не досчитывается.
    for (const ExactScene& scene : exactScenes()) {
        if (std::strcmp(scene.key, "chain100") != 0) print(scene.key, "точный   ", after(scene.build, nullptr, frames, false, 64));
        print(scene.key, "быстрый  ", after(scene.build, nullptr, frames, true));
    }
}
