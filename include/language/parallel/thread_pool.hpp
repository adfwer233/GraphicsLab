#pragma once
#include "spdlog/spdlog.h"

#include <functional>
#include <future>
#include <queue>
#include <thread>
#include <type_traits>

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

    std::mutex queueMutex;
    std::condition_variable task_available_condition, all_stopped_condition;

    void worker_loop(const std::stop_token &stoken) {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock lock(queueMutex);
                task_available_condition.wait(lock,
                                              [this, stoken] { return !tasks.empty() or stoken.stop_requested(); });

                if (stoken.stop_requested() or tasks.empty()) {
                    all_stopped_condition.notify_one();
                    return;
                }

                task = std::move(tasks.front());
                tasks.pop();
            }

            task();
        }
    }

    void stop() {
        {
            std::unique_lock lock(queueMutex);
            all_stopped_condition.wait(lock, [this] { return tasks.empty(); });

            if (not workers.empty()) {
                for (auto &worker : workers) {
                    worker.request_stop();
                }
            }
        }

        task_available_condition.notify_all();
    }
};