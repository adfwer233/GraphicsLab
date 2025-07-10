#include <gtest/gtest.h>

#include "language/parallel/thread_pool.hpp"
#include "spdlog/spdlog.h"

TEST(ThreadPoolTest, ThreadPoolSimpleTest1) {
    constexpr int num_tasks = 10;
    spdlog::set_level(spdlog::level::debug);
    spdlog::debug("test");
    spdlog::debug("hardware concurrency: {}", std::jthread::hardware_concurrency());

    ThreadPool pool(std::jthread::hardware_concurrency());

    std::vector<std::future<int>> results;

    spdlog::debug("submit tasks...");

    for (int i = 0; i < num_tasks; i++) {
        results.push_back(pool.enqueue([i]() -> int {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::size_t tid = std::hash<std::thread::id>{}(std::this_thread::get_id());
            spdlog::debug("Task {} is running in thread {}", i, tid);
            return i * i;
        }));
    }

    spdlog::debug("All tasks submitted. Waiting for results...\n");

    for (int i = 0; i < num_tasks; i++) {
        int value = results[i].get();
        spdlog::debug("Result of task {}: {}", i, value);
    }
}