#pragma once

#include <vector>

// Primary template - assumes it's not a vector
template<typename T>
struct is_vector : std::false_type {};

// Specialization for std::vector
template<typename T, typename Alloc>
struct is_vector<std::vector<T, Alloc>> : std::true_type {};

// Helper variable template to simplify usage
template<typename T>
inline constexpr bool is_vector_v = is_vector<T>::value;