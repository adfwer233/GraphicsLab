#pragma once

#include "sol/sol.hpp"
#include "vkl/scene_tree/vkl_mesh_types.hpp"
#include "vkl/scene_tree/vkl_scene_tree.hpp"

namespace GraphicsLab {

struct LuaSceneInterface {

    void add_point_cloud_2d(sol::table tbl) {
        std::vector<glm::vec2> points;
        for (std::size_t i = 1; i <= tbl.size(); ++i) {
            glm::vec2 p = tbl[i];
            points.push_back(p);
        }
        for (auto& p : points) {
            std::cout << "point = (" << p.x << ", " << p.y << ")\n";
        }
    }

    static void bind(sol::state& lua) {
        lua.new_usertype<LuaSceneInterface>("LuaSceneInterface",
            "add_point_cloud_2d", &LuaSceneInterface::add_point_cloud_2d);
    }

private:
    SceneTree::VklSceneTree *scene_ = nullptr;
};

}