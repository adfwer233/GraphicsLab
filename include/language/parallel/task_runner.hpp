#pragma once
#include "thread_pool.hpp"

namespace Parallel {

struct GridIndex {
    int i, j, k;
};

struct Grid {
    int size_x = 1;
    int size_y = 1;
    int size_z = 1;

    [[nodiscard]] size_t index(const int i, const int j, const int k) const {
        return i * size_y * size_z + j * size_z + k;
    }

    [[nodiscard]] size_t size() const { return size_x * size_y * size_z; }

    [[nodiscard]] auto create_indices() const -> std::vector<GridIndex> {
        std::vector<GridIndex> indices(size_x * size_y * size_z);
        for (int i = 0; i < size_x; i++) {
            for (int j = 0; j < size_y; j++) {
                for (int k = 0; k < size_z; k++) {
                    auto idx = index(i, j, k);
                    indices[idx].i = i;
                    indices[idx].j = j;
                    indices[idx].k = k;
                }
            }
        }
        return indices;
    };
};

template<typename T>
struct TaskRunner {
    explicit TaskRunner(ThreadPool& pool) : pool_(pool) {
    }

    /**
     * @brief run all tasks, implemenation of cases that the return type is not void
     *
     * @tparam Func
     * @param items
     * @param func
     * @return a vector of results
     */
    template <typename Func> requires (not std::same_as<std::invoke_result_t<Func, T>, void>)
    auto run_all(const std::vector<T>& items, Func&& func) -> std::vector<std::invoke_result_t<Func, T>> {
        using ReturnType = std::invoke_result_t<Func, T>;
        std::vector<std::future<ReturnType>> futures;

        for (const auto& item: items) {
            futures.push_back(pool_.enqueue([func, item]() -> ReturnType { return func(item); }));
        }

        std::vector<ReturnType> results;
        results.reserve(futures.size());

        for (auto& future: futures)
            results.push_back(future.get());

        return results;
    }

    /**
    * @brief run all tasks, implemenation of cases that the return type is void
    *
    * @tparam Func
    * @param items
    * @param func
    * @return a vector of results
    */
   template <typename Func> requires (std::same_as<std::invoke_result_t<Func, T>, void>)
   auto run_all(const std::vector<T>& items, Func&& func) -> void {
        using ReturnType = std::invoke_result_t<Func, T>;
        std::vector<std::future<ReturnType>> futures;

        for (const auto& item: items) {
            futures.push_back(pool_.enqueue([func, item]() -> ReturnType { return func(item); }));
        }

        for (auto& future: futures)
            future.get();
    }


private:
    ThreadPool& pool_;
};

}