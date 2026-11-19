#pragma once
// Пул потоков для независимых заданий.
//
// run(count, job) вызывает job(0) … job(count − 1) — каждое ровно один раз, в неизвестном порядке и в
// разных потоках — и возвращается, когда все закончились. Задания разбираются по одному через общий
// счётчик: поток, которому досталось тяжёлое задание, не задерживает остальные, и нагрузка
// выравнивается сама. Ждать приходится только тех, кто успел взять задание: поток, проснувшийся
// после конца круга, в него не входит — иначе кадр стоял бы, пока система будит заснувшее ядро
// (замер study-frame-times: выбросы до 9 мс при шаге 1,6 мс). Ответ движка от числа потоков зависеть не должен, поэтому задание пишет только в
// своё место, а склеивает результаты вызывающий — в порядке номеров.

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace kp {

class WorkerPool {
public:
    // Общий пул процесса; потоков — по числу ядер, пока не задано иное.
    static WorkerPool& shared();
    explicit WorkerPool(int threads);
    ~WorkerPool();
    WorkerPool(const WorkerPool&) = delete;
    WorkerPool& operator=(const WorkerPool&) = delete;

    int threads() const { return int(workers_.size()) + 1; }
    // Число потоков вместе с вызывающим; один — задания идут в вызывающем потоке по порядку.
    void resize(int threads);
    void run(size_t count, const std::function<void(size_t)>& job);

private:
    void work();
    void drain();
    void stop();

    std::vector<std::thread> workers_;
    std::mutex mutex_;
    std::condition_variable wake_, finished_;
    const std::function<void(size_t)>* job_ = nullptr;
    size_t count_ = 0, busy_ = 0, round_ = 0; // busy_ — потоков, разбирающих задания круга
    std::atomic<size_t> next_{0};
    bool open_ = false, stopping_ = false;     // open_ — круг ещё принимает потоки
};

} // namespace kp
