#pragma once

#include <map>

// Primary template - assumes it's not a map
template <typename T> struct is_map : std::false_type {};

// Specialization for std::map
template <typename Key, typename T, typename Compare, typename Alloc>
struct is_map<std::map<Key, T, Compare, Alloc>> : std::true_type {};

// Helper variable template to simplify usage
template <typename T> inline constexpr bool is_map_v = is_map<T>::value;