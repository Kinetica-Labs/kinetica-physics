// Прибор памяти: сколько раз за кадр быстрый шаг обращается к куче и откуда.
//   study-allocations [имя сцены [секунд]]        по умолчанию все стандартные сцены, время сцены
// Глобальные operator new и delete этой программы подменены счётчиками (в библиотеке их никто не
// подменяет). Печатает обращений к куче и килобайт на кадр. С KP_SITES=1 каждое 64-е обращение
// записывает цепочку вызовов, и после сцены печатаются функции движка, из которых память просят
// чаще всего. KP_THREADS — число потоков пула.

#include <math/WorkerPool.h>
#include "../../examples/scenes/StandardScenes.h"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cxxabi.h>
#include <dlfcn.h>
#include <execinfo.h>
#include <map>
#include <new>
#include <string>
#include <vector>

namespace {

constexpr int kDepth = 14, kSamples = 1 << 17, kEvery = 64;
std::atomic<long long> calls{0}, bytes{0};
std::atomic<int> sampled{0};
std::atomic<bool> sampling{false};
void* chains[kSamples][kDepth];
int lengths[kSamples];

void* counted(std::size_t size) {
    const long long number = calls.fetch_add(1, std::memory_order_relaxed);
    bytes.fetch_add(static_cast<long long>(size), std::memory_order_relaxed);
    if (sampling.load(std::memory_order_relaxed) && number % kEvery == 0) {
        const int slot = sampled.fetch_add(1, std::memory_order_relaxed);
        if (slot < kSamples) lengths[slot] = backtrace(chains[slot], kDepth);
    }
    if (void* memory = std::malloc(size ? size : 1)) return memory;
    throw std::bad_alloc();
}

// Имя первой функции движка в цепочке вызовов (пространство имён kp).
std::string siteOf(void* const* chain, int length) {
    for (int k = 1; k < length; ++k) {
        Dl_info info{};
        if (!dladdr(chain[k], &info) || !info.dli_sname) continue;
        int status = 0;
        char* plain = abi::__cxa_demangle(info.dli_sname, nullptr, nullptr, &status);
        const std::string name = status == 0 && plain ? plain : info.dli_sname;
        std::free(plain);
        if (name.rfind("kp::", 0) == 0) return name.substr(0, name.find('('));
    }
    return "(вне движка)";
}

void printSites() {
    const int count = std::min(sampled.load(), kSamples);
    std::map<std::string, int> sites;
    for (int k = 0; k < count; ++k) ++sites[siteOf(chains[k], lengths[k])];
    std::vector<std::pair<int, std::string>> order;
    for (const auto& [name, hits] : sites) order.push_back({hits, name});
    std::sort(order.rbegin(), order.rend());
    for (size_t k = 0; k < order.size() && k < 12; ++k) std::printf("    %5.1f %%  %s\n", 100.0 * order[k].first / count, order[k].second.c_str());
}

} // namespace

void* operator new(std::size_t size) { return counted(size); }
void* operator new[](std::size_t size) { return counted(size); }
void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete[](void* memory) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t) noexcept { std::free(memory); }
void operator delete[](void* memory, std::size_t) noexcept { std::free(memory); }

int main(int argc, char** argv) {
    using namespace kp;
    if (const char* threads = std::getenv("KP_THREADS")) WorkerPool::shared().resize(std::atoi(threads));
    void* warm[4];
    backtrace(warm, 4); // первая раскрутка стека сама просит память
    const std::string only = argc > 1 ? argv[1] : "";
    std::printf("%-10s %7s %16s %12s\n", "сцена", "кадров", "обращений/кадр", "КБ/кадр");
    for (const StandardScene& scene : standardScenes()) {
        if (!only.empty() && only != scene.key) continue;
        World world = scene.build();
        world.realtime = true;
        const int frames = int(60 * (argc > 2 ? std::atof(argv[2]) : scene.seconds));
        calls = 0, bytes = 0, sampled = 0, sampling = std::getenv("KP_SITES") != nullptr;
        for (int frame = 0; frame < frames; ++frame) {
            if (scene.control) scene.control(world, frame / 60.0);
            world.step(Real(1) / 60);
        }
        sampling = false;
        const long long total = calls.load(), size = bytes.load();
        std::printf("%-10s %7d %16.0f %12.0f\n", scene.key, frames, double(total) / frames, double(size) / frames / 1024);
        if (std::getenv("KP_SITES")) printSites();
        std::fflush(stdout);
    }
    return 0;
}
