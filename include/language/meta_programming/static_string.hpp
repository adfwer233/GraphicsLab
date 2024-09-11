#pragma once

#include <string_view>

namespace MetaProgramming {

template <char... args> struct StaticString {
    static constexpr const char str[] = {args...};
    operator const char *() const {
        return StaticString::str;
    }
};

} // namespace MetaProgramming