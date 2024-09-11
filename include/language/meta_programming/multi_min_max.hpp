#pragma once

#include <algorithm>
#include <concepts>

namespace MetaProgramming {
namespace detail {
template <typename T> T min_impl(T first) {
    return first;
}

template <typename T, typename... Args> T min_impl(T arg1, Args... args) {
    return std::min(arg1, min_impl(args...));
}

template <typename T> T max_impl(T first) {
    return first;
}

template <typename T, typename... Args> T max_impl(T arg1, Args... args) {
    return std::max(arg1, max_impl(args...));
}
} // namespace detail

template <typename T, typename... Args> T min(T first, Args... args) {
    return detail::min_impl(first, args...);
}

template <typename T, typename... Args> T max(T first, Args... args) {
    return detail::max_impl(first, args...);
}
} // namespace MetaProgramming