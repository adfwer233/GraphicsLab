#pragma once
#include "spdlog/spdlog.h"

#include <functional>
#include <future>
#include <queue>
#include <thread>
#include <type_traits>

namespace Parallel {
struct ThreadPool {

    explicit ThreadPool(size_t thread_count) {
        for (size_t i = 0; i < thread_count; ++i) {
            workers.emplace_back([this](const std::stop_token &st) { this->worker_loop(st); });
        }
    }

    ~ThreadPool() {
        stop();
    }

    template <typename Func, typename... Args>
    auto enqueue(Func &&f, Args &&...args) -> std::future<std::invoke_result_t<Func, Args...>> {
        using return_type = std::invoke_result_t<Func, Args...>;

        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<Func>(f), std::forward<Args>(args)...));

        std::future<return_type> result = task->get_future();

        {
            std::scoped_lock lock(queueMutex);
            tasks.emplace([task]() { (*task)(); });
        }

        task_available_condition.notify_one();
        return result;
    }

  private:
    std::vector<std::jthread> workers;
    std::queue<std::function<void()>> tasks;
    bool stopping = false;

    std::mutex queueMutex;
    std::condition_variable task_available_condition, all_stopped_condition;

    void worker_loop(std::stop_token stoken) {
        while (true) {
            std::function<void()> task;

            {
                std::unique_lock lock(queueMutex);
                task_available_condition.wait(lock, [this, &stoken] { return stopping || !tasks.empty(); });

                if (stopping && tasks.empty())
                    return;

                task = std::move(tasks.front());
                tasks.pop();
            }

            task();
        }
    }

    void stop() {
        {
            std::lock_guard lock(queueMutex);
            stopping = true;
        }

        task_available_condition.notify_all();

        // jthread destructor will now safely join
    }
};
} // namespace Parallel