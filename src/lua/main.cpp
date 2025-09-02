#include "sol/sol.hpp"
#include <iostream>

#include "lua/lua_binding.hpp"

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

    // Bind the Scene class
    lua.new_usertype<Scene>("Scene", "add_light", &Scene::add_light);

    // Create an instance in C++
    Scene scene;

    // Pass it into Lua as a global
    lua["scene"] = &scene;

    // Run Lua script
    lua.script(R"(
        scene:add_light("MainLight")
    )");

    return 0;
}
