#include "sol/sol.hpp"
#include <iostream>

#include "lua/lua_binding.hpp"
#include "lua/scene_binding.hpp"

class Scene {
  public:
    void add_light(const std::string &name) {
        std::cout << "Added light: " << name << "\n";
    }
};

int main() {
    sol::state lua;
    lua.open_libraries(sol::lib::base);

    GraphicsLab::LuaBinding::bind(lua);
    GraphicsLab::LuaSceneInterface::bind(lua);
    // Bind the Scene class
    lua.new_usertype<Scene>("Scene", "add_light", &Scene::add_light);

    // Create an instance in C++
    Scene scene;
    GraphicsLab::LuaSceneInterface sceneInterface;

    // Pass it into Lua as a global
    lua["scene"] = &scene;
    lua["sceneInterface"] = &sceneInterface;

    // Run Lua script
    lua.script(R"(
        scene:add_light("MainLight")
    )");

    lua.script(R"(
        local points = {
            vec2.new(1.0, 2.0),
            vec2.new(3.0, 4.0),
            vec2.new(5.0, 6.0)
        }

        local v3 = vec3.new(1,2,3)
        print("vec3:", v3.x, v3.y, v3.z)

        local v4 = vec4.new(1,2,3,4)
        print("vec4:", v4.x, v4.y, v4.z, v4.w)

        sceneInterface:add_point_cloud_2d(points)
    )");

    return 0;
}
