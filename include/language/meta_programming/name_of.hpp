#pragma once

#include <array>
#include <concepts>

#include "static_for_loop.hpp"
#include "static_string.hpp"

#if defined(_MSC_VER)

namespace MetaProgramming {
/**
 * ref to https://wnaabi.readthedocs.io/en/latest/PrettyFunction.html
 * @tparam T
 * @return
 */
template <typename T> auto type_name_of() {
    constexpr std::string_view full_name = __FUNCSIG__;
    constexpr auto begin = full_name.find_first_of('<') + 1;
    constexpr auto end = full_name.find_first_of('>');
    constexpr auto type_name_view = full_name.substr(begin, end - begin);
    constexpr auto indices = std::make_index_sequence<type_name_view.size()>();
    constexpr auto type_name = [&]<std::size_t... indices>(std::integer_sequence<std::size_t, indices...>) {
        constexpr auto str = StaticString<type_name_view[indices]..., '\0'>();
        return str;
    }(indices);
    return type_name;
}

template <typename T, T value> auto type_name_of() {
    constexpr std::string_view full_name = __FUNCSIG__;
    constexpr auto begin = full_name.find_first_of(',') + 1;
    constexpr auto end = full_name.find_first_of('>');
    constexpr auto type_name_view = full_name.substr(begin, end - begin);
    constexpr auto indices = std::make_index_sequence<type_name_view.size()>();
    constexpr auto type_name = [&]<std::size_t... indices>(std::integer_sequence<std::size_t, indices...>) {
        constexpr auto str = StaticString<type_name_view[indices]..., '\0'>();
        return str;
    }(indices);
    return type_name;
}

template <typename T>
concept IsEnum = std::is_enum_v<T>;

template <IsEnum T, T value> consteval bool is_member_of_enum() {
    constexpr std::string_view full_name = __FUNCSIG__;
    constexpr auto begin = full_name.find_first_of(',') + 1;
    constexpr auto end = full_name.find_first_of('>');
    constexpr auto type_name_view = full_name.substr(begin, end - begin);
    return type_name_view[0] != '(';
}

template <IsEnum T> auto all_name_of_enum() {
    constexpr auto indices = std::make_index_sequence<256>();
    constexpr auto total_valid = [&]<std::size_t... indices>(std::integer_sequence<std::size_t, indices...>) {
        int n = 0;
        ((n += is_member_of_enum<T, static_cast<T>(indices)>() ? 1 : 0), ...);
        return n;
    }(indices);
    std::array<std::string_view, total_valid> result;

    int n = 0;
    auto static_part = [&]<size_t value>(std::integral_constant<size_t, value>) {
        if constexpr (is_member_of_enum<T, static_cast<T>(value)>()) {
            result[n] = type_name_of<T, static_cast<T>(value)>().str;
            n++;
        }
    };

    auto update_result = [&]<std::size_t... indices>(std::integer_sequence<std::size_t, indices...>) {
        (static_part(std::integral_constant<size_t, indices>()), ...);
    };
    update_result(indices);

    return result;
}
} // namespace MetaProgramming

#endif