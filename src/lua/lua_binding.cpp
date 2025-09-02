#include "lua/lua_binding.hpp"

#include "geometry/parametric/curve_type_list.hpp"

namespace GraphicsLab {

void LuaBinding::bind(sol::state &lua) {

    lua.new_usertype<Geometry::BezierCurve2D>("BezierCurve2D", "evaluate", &Geometry::BezierCurve2D::evaluate);
}

} // namespace GraphicsLab