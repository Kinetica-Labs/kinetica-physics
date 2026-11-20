#include <math/StepMemory.h>

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <new>

namespace kp {
namespace {

constexpr size_t kHeader = 16;          // перед каждым куском: откуда он взят
constexpr size_t kBlock = size_t(1) << 22; // запас растёт блоками по 4 МБ
constexpr char kFromHeap = 0, kFromStep = 1;

// Запас одного потока: блоки, взятые у кучи один раз и служащие всем шагам.
struct Reserve {
    std::vector<char*> blocks;
    std::vector<size_t> sizes;
    size_t block = 0, used = 0;
    bool serving = false; // запас потока пула

    Reserve();
    ~Reserve();
    void* take(size_t bytes) {
        while (block < blocks.size() && used + bytes > sizes[block]) ++block, used = 0;
        if (block == blocks.size()) {
            const size_t size = bytes > kBlock ? bytes : kBlock;
            blocks.push_back(static_cast<char*>(std::malloc(size))), sizes.push_back(size);
            if (!blocks.back()) throw std::bad_alloc();
        }
        void* memory = blocks[block] + used;
        used += bytes;
        return memory;
    }
    void rewind(bool wipe) {
        for (size_t k = 0; wipe && k < blocks.size(); ++k) std::memset(blocks[k], 0xFF, sizes[k]);
        block = 0, used = 0;
    }
    size_t held() const {
        size_t bytes = used;
        for (size_t k = 0; k < block; ++k) bytes += sizes[k];
        return bytes;
    }
};

// Все запасы процесса: когда закрыт последний StepMemory, возвращаются запасы потоков пула — они стоят.
struct Registry {
    std::mutex mutex;
    std::vector<Reserve*> reserves;
    std::atomic<int> open{0}; // сколько StepMemory открыто во всех потоках; меняется под mutex
    bool wipe = false;
};
Registry& registry() {
    static Registry* all = new Registry; // не разрушается: запасы потоков отписываются при выходе
    return *all;
}

Reserve::Reserve() {
    const std::lock_guard<std::mutex> lock(registry().mutex);
    registry().reserves.push_back(this);
}
Reserve::~Reserve() {
    const std::lock_guard<std::mutex> lock(registry().mutex);
    std::vector<Reserve*>& all = registry().reserves;
    for (size_t k = 0; k < all.size(); ++k)
        if (all[k] == this) all[k] = all.back(), all.pop_back();
    for (char* each : blocks) std::free(each);
}

thread_local Reserve reserve;
thread_local int depth = 0; // сколько StepMemory открыто этим потоком

} // namespace

StepMemory::StepMemory() {
    Registry& all = registry();
    const std::lock_guard<std::mutex> lock(all.mutex); // новый шаг не начнётся, пока возвращаются запасы пула
    if (all.open.fetch_add(1) == 0) {
        const char* mode = std::getenv("KP_STEP_MEMORY");
        all.wipe = mode && std::strcmp(mode, "check") == 0;
    }
    ++depth;
}

StepMemory::~StepMemory() {
    Registry& all = registry();
    // Свой запас: в нём данные одного этого шага, другим потокам он не виден.
    if (--depth == 0 && !reserve.serving) reserve.rewind(all.wipe);
    const std::lock_guard<std::mutex> lock(all.mutex);
    if (all.open.fetch_sub(1) != 1) return;
    for (Reserve* each : all.reserves)
        if (each->serving) each->rewind(all.wipe);
}

void* StepMemory::take(size_t bytes) {
    const size_t size = (bytes + kHeader + 15) & ~size_t(15);
    const bool stepped = depth > 0 || (reserve.serving && registry().open.load(std::memory_order_relaxed) > 0);
    char* memory = stepped ? static_cast<char*>(reserve.take(size)) : static_cast<char*>(::operator new(size));
    memory[0] = stepped ? kFromStep : kFromHeap;
    return memory + kHeader;
}

size_t StepMemory::held() { return reserve.held(); }

void StepMemory::serve() { reserve.serving = true; }

void StepMemory::give(void* memory) noexcept {
    char* start = static_cast<char*>(memory) - kHeader;
    if (start[0] == kFromHeap) ::operator delete(start);
}

} // namespace kp
