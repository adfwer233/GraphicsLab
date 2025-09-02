#pragma once

#include "sol/sol.hpp"

namespace GraphicsLab {

struct LuaBinding {
    static void bind(sol::state& lua);
};

}