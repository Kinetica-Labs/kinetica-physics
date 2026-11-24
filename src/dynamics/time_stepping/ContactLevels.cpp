#include <dynamics/time_stepping/ContactLevels.h>

#include <math/WorkerPool.h>

#include <algorithm>
#include <atomic>
#include <thread>

namespace kp {
namespace {

constexpr size_t kParallelContacts = 1024; // меньше — будить потоки дороже, чем обойти самому
constexpr size_t kMinWidth = 8;            // контактов на уровень в среднем: у́же — потокам нечего делить
constexpr size_t kChunk = 2;               // контактов за одно обращение потока к счётчику уровня; замер: 1 и 4 хуже на 3 %, 16 — на 38 %

// Сколько кусков уровня разобрано и сколько закончено.
struct Progress {
    std::atomic<size_t> taken{0}, done{0};
};

// Ожидание конца уровня длится доли микросекунды, поэтому поток не засыпает. Если соседа сняла с ядра
// система, ждать пришлось бы долго — тогда поток уступает ядро.
void relax(size_t spins) {
    if (spins % 1024 == 1023) return std::this_thread::yield();
#if defined(__x86_64__) || defined(__i386__)
    __builtin_ia32_pause();
#endif
}

} // namespace

void ContactLevels::each(const StepArray<SoftContact>& contacts, const StepArray<SolverBody>& states, const std::function<void(size_t)>& job) {
    const size_t count = contacts.size();
    if (WorkerPool::shared().threads() > 1 && count >= kParallelContacts) {
        if (order_.size() != count) build(contacts, states); // контакты за шаг только дописываются
        if (count >= kMinWidth * (first_.size() - 1)) return together(contacts, states, job);
    }
    for (size_t c = 0; c < count; ++c) SoftContact::prefetch(contacts, states, c + SoftContact::kNear, c + SoftContact::kFar), job(c);
}

void ContactLevels::build(const StepArray<SoftContact>& contacts, const StepArray<SolverBody>& states) {
    const size_t count = contacts.size();
    StepArray<size_t> next(states.size(), 0), level(count); // next — уровень следующего контакта тела
    size_t depth = 0;
    for (size_t c = 0; c < count; ++c) {
        const SoftContact::NormalRow row = contacts[c].normalRow(0);
        const size_t a = size_t(row.a), b = size_t(row.b);
        level[c] = std::max(states[a].inverseMass != 0 ? next[a] : 0, states[b].inverseMass != 0 ? next[b] : 0);
        next[a] = next[b] = level[c] + 1, depth = std::max(depth, level[c] + 1);
    }
    first_.assign(depth + 1, 0);
    for (size_t at : level) ++first_[at + 1];
    for (size_t k = 1; k <= depth; ++k) first_[k] += first_[k - 1];
    StepArray<size_t> place(first_.begin(), first_.end() - 1);
    order_.resize(count);
    for (size_t c = 0; c < count; ++c) order_[place[level[c]]++] = c;
}

// Каждый поток идёт по уровням от первого. Куски уровня разбираются через счётчик; к следующему уровню
// поток переходит, когда закончены все куски этого. Опоздавший поток пробегает разобранные уровни, не
// найдя в них работы, — ждать его не приходится, а вызывающий поток в одиночку закончил бы всё сам.
void ContactLevels::together(const StepArray<SoftContact>& contacts, const StepArray<SolverBody>& states, const std::function<void(size_t)>& job) const {
    const size_t levels = first_.size() - 1, count = order_.size();
    StepArray<Progress> progress(levels);
    const auto at = [&](size_t i) { return i < count ? order_[i] : count; };
    WorkerPool::shared().run(size_t(WorkerPool::shared().threads()), [&](size_t) {
        for (size_t level = 0; level < levels; ++level) {
            const size_t begin = first_[level], end = first_[level + 1], chunks = (end - begin + kChunk - 1) / kChunk;
            Progress& now = progress[level];
            for (size_t k = now.taken.fetch_add(1); k < chunks; k = now.taken.fetch_add(1)) {
                for (size_t i = begin + k * kChunk; i < std::min(end, begin + (k + 1) * kChunk); ++i)
                    SoftContact::prefetch(contacts, states, at(i + SoftContact::kNear), at(i + SoftContact::kFar)), job(order_[i]);
                now.done.fetch_add(1);
            }
            for (size_t spins = 0; now.done.load() < chunks; ++spins) relax(spins);
        }
    });
}

} // namespace kp
