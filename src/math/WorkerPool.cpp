#include <math/WorkerPool.h>

#include <math/StepMemory.h>

#include <algorithm>

namespace kp {

WorkerPool& WorkerPool::shared() {
    static WorkerPool pool(int(std::max(1u, std::thread::hardware_concurrency())));
    return pool;
}

WorkerPool::WorkerPool(int threads) { resize(threads); }

WorkerPool::~WorkerPool() { stop(); }

void WorkerPool::stop() {
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    wake_.notify_all();
    for (std::thread& worker : workers_) worker.join();
    workers_.clear(), stopping_ = false;
}

void WorkerPool::resize(int threads) {
    stop();
    for (int k = 1; k < threads; ++k) workers_.emplace_back([this] { work(); });
}

// Поток ждёт нового круга заданий и входит в него, только если круг ещё открыт.
void WorkerPool::work() {
    StepMemory::serve();
    size_t seen = 0;
    for (;;) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            wake_.wait(lock, [&] { return stopping_ || round_ != seen; });
            if (stopping_) return;
            seen = round_;
            if (!open_) continue;
            ++busy_;
        }
        drain();
        const std::lock_guard<std::mutex> lock(mutex_);
        if (--busy_ == 0) finished_.notify_one();
    }
}

void WorkerPool::drain() {
    for (size_t i = next_.fetch_add(1); i < count_; i = next_.fetch_add(1)) (*job_)(i);
}

// Будится не больше потоков, чем заданий сверх одного: одно берёт вызывающий.
void WorkerPool::run(size_t count, const std::function<void(size_t)>& job) {
    if (workers_.empty() || count < 2) {
        for (size_t i = 0; i < count; ++i) job(i);
        return;
    }
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        job_ = &job, count_ = count, next_ = 0, open_ = true, ++round_;
    }
    if (count > workers_.size()) wake_.notify_all();
    else for (size_t k = 1; k < count; ++k) wake_.notify_one();
    drain();
    std::unique_lock<std::mutex> lock(mutex_);
    open_ = false;
    finished_.wait(lock, [&] { return busy_ == 0; });
}

} // namespace kp
