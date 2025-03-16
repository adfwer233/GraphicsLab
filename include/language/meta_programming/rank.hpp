#pragma once

namespace MetaProgramming {

template <int N = 64> struct Rank : Rank<N - 1> {};

template <> struct Rank<0> {};

} // namespace MetaProgramming