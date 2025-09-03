#include "lua/lua_binding.hpp"
#include "geometry/parametric/curve_type_list.hpp"

namespace GraphicsLab {

void LuaBinding::bind(sol::state &lua) {

    // bind GLM vecs

    // Register glm::vec2
    lua.new_usertype<glm::vec2>("vec2",
        sol::constructors<glm::vec2(), glm::vec2(float,float)>(),
        "x", &glm::vec2::x,
        "y", &glm::vec2::y
    );

    // Register glm::vec3
    lua.new_usertype<glm::vec3>("vec3",
        sol::constructors<glm::vec3(), glm::vec3(float,float,float)>(),
        "x", &glm::vec3::x,
        "y", &glm::vec3::y,
        "z", &glm::vec3::z
    );

    // Register glm::vec4
    lua.new_usertype<glm::vec4>("vec4",
        sol::constructors<glm::vec4(), glm::vec4(float,float,float,float)>(),
        "x", &glm::vec4::x,
        "y", &glm::vec4::y,
        "z", &glm::vec4::z,
        "w", &glm::vec4::w
    );

    lua.new_usertype<glm::vec2>("vec2",
        sol::constructors<glm::vec2(), glm::vec2(float,float)>(),
        "x", &glm::vec2::x,
        "y", &glm::vec2::y
    );

    lua.new_usertype<Geometry::BezierCurve2D>("BezierCurve2D", "evaluate", &Geometry::BezierCurve2D::evaluate);
}

} // namespace GraphicsLab