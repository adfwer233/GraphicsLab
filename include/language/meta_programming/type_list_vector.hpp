#pragma once

#include "language/coroutine/generator.hpp"
#include "type_list.hpp"
#include <vector>

namespace MetaProgramming {

/**
 * TypeListVector is a simple implementation of heterogeneous vector
 *
 * data are stored in a tuple of vectors: std::tuple<std::vector<T1> , ..., std::vector<Tn>>
 *
 * @tparam ts types of the TypeListVector
 */
template <typename... ts> struct TypeListVector {

    using type_list_type = TypeList<ts...>;

    using tuple_type = type_list_type::template monad<std::vector>::template to<std::tuple>;

    tuple_type data;

    template <typename T> void push_back(T &item) {
        static_assert(MetaProgramming::TypeListFunctions::IsAnyOf<type_list_type, T>::value);
        constexpr size_t index = MetaProgramming::TypeListFunctions::IndexOf<type_list_type, T>::value;
        auto &vec = std::get<index>(data);
        vec.push_back(item);
    }

    template <typename T> void push_back(T &&item) {
        static_assert(MetaProgramming::TypeListFunctions::IsAnyOf<type_list_type, T>::value);
        constexpr size_t index = MetaProgramming::TypeListFunctions::IndexOf<type_list_type, T>::value;
        auto &vec = std::get<index>(data);
        vec.push_back(std::forward<T>(item));
    }

    template <typename T> Generator<T> traverse() {
        static_assert(MetaProgramming::TypeListFunctions::IsAnyOf<type_list_type, T>::value);
        constexpr size_t index = MetaProgramming::TypeListFunctions::IndexOf<type_list_type, T>::value;
        auto &vec = std::get<index>(data);
        for (auto item : vec) {
            co_yield item;
        }
    }

    template <uint32_t index>
    Generator<typename MetaProgramming::TypeListFunctions::KthOf<type_list_type, index>::type> traverse_by_index() {
        auto &vec = std::get<index>(data);
        for (auto item : vec) {
            co_yield item;
        }
    }

    template <typename T> Generator<T> traverse_as() {
        return traverse_as_detail<T, type_list_type::size - 1>();
    }

    template <typename T> T &get(size_t item_index) {
        static_assert(MetaProgramming::TypeListFunctions::IsAnyOf<type_list_type, T>::value);
        constexpr size_t index = MetaProgramming::TypeListFunctions::IndexOf<type_list_type, T>::value;
        auto &vec = std::get<index>(data);
        return vec[item_index];
    }

    template <typename T> T get(size_t item_index) const {
        static_assert(MetaProgramming::TypeListFunctions::IsAnyOf<type_list_type, T>::value);
        constexpr size_t index = MetaProgramming::TypeListFunctions::IndexOf<type_list_type, T>::value;
        auto &vec = std::get<index>(data);
        return vec[item_index];
    }

  private:
    template <typename target_type, size_t current_index> Generator<target_type> traverse_as_detail() {
        for (auto item : std::get<current_index>(data)) {
            co_yield reinterpret_cast<target_type>(item);
        }

        if constexpr (current_index > 0) {
            for (auto item : traverse_as_detail<target_type, current_index - 1>()) {
                co_yield item;
            }
        }
    }
};

} // namespace MetaProgramming