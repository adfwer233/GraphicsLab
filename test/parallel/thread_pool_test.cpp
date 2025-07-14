#include "language/parallel/task_runner.hpp"

#include <gtest/gtest.h>

#include "language/parallel/thread_pool.hpp"
#include "spdlog/spdlog.h"

TEST(ThreadPoolTest, ThreadPoolSimpleTest1) {
    constexpr int num_tasks = 10;
    spdlog::set_level(spdlog::level::debug);
    spdlog::debug("test");
    spdlog::debug("hardware concurrency: {}", std::jthread::hardware_concurrency());

    Parallel::ThreadPool pool(std::jthread::hardware_concurrency());

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

TEST(ThreadPoolTest, TaskRunnerTest1) {
    constexpr int M = 10, N = 10, L = 10;
    spdlog::set_level(spdlog::level::debug);

    using namespace Parallel;

    ThreadPool pool(std::jthread::hardware_concurrency());
    Grid grid{M, N, L};
    auto indices = grid.create_indices();
    TaskRunner<GridIndex> runner(pool);

    auto result = runner.run_all(indices,
        [](const GridIndex& index) -> int {
            return index.i + index.j + index.k;
        }
    );

    for (int i = 0; i < M; i++) {
        for (int j = 0; j < N; j++) {
            for (int k = 0; k < L; k++) {
                EXPECT_EQ(result[grid.index(i, j, k)], i + j + k);
            }
        }
    }
}

TEST(ThreadPoolTest, TaskRunnerReturnVoidTest) {
    constexpr int M = 10, N = 10, L = 10;
    spdlog::set_level(spdlog::level::debug);

    using namespace Parallel;

    ThreadPool pool(std::jthread::hardware_concurrency());
    Grid grid{M, N, L};
    auto indices = grid.create_indices();
    TaskRunner<GridIndex> runner(pool);

    runner.run_all(indices,
        [](const GridIndex& index) -> void {
            int x = index.i + index.j + index.k;
        }
    );
}