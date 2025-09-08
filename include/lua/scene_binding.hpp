#pragma once

#include "geometry/parametric/tessellator.hpp"
#include "sol/sol.hpp"
#include "vkl/scene_tree/vkl_mesh_types.hpp"
#include "vkl/scene_tree/vkl_scene_tree.hpp"

namespace GraphicsLab {

struct LuaSceneInterface {

    explicit LuaSceneInterface(SceneTree::VklSceneTree *scene) : scene_(scene) {
    }

    void add_point_cloud_2d(sol::table tbl) {
        std::vector<glm::vec2> points;
        for (std::size_t i = 1; i <= tbl.size(); ++i) {
            glm::vec2 p = tbl[i];
            points.push_back(p);
        }

        PointCloud2D pointcloud;
        for (auto &p : points) {
            PointCloud2D::vertex_type vertex;
            vertex.position = p;
            vertex.color = glm::vec3(1.0, 0.0, 0.0);
            std::cout << "point = (" << p.x << ", " << p.y << ")\n";
            pointcloud.vertices.push_back(vertex);
        }

        auto pc = scene_->addGeometryNode<PointCloud2D>(std::move(pointcloud), "pointcloud");
    }

    void add_bezier_curve_2d(sol::table tbl) {
        std::vector<glm::dvec2> points;
        for (std::size_t i = 1; i <= tbl.size(); ++i) {
            glm::vec2 p = tbl[i];
            points.emplace_back(p);
        }

        Geometry::BezierCurve2D curve_2d(std::move(points));

        Geometry::Tessellator::tessellate(curve_2d);
        CurveMesh2D curve_mesh = *curve_2d.mesh;
        auto bezier = scene_->addGeometryNode<CurveMesh2D>(std::move(curve_mesh), "bezier curve");
    }

    static void bind(sol::state &lua) {
        lua.new_usertype<LuaSceneInterface>("LuaSceneInterface", "add_point_cloud_2d",
                                            &LuaSceneInterface::add_point_cloud_2d,
                                            "add_bezier_curve_2d", &LuaSceneInterface::add_bezier_curve_2d);
    }

  private:
    SceneTree::VklSceneTree *scene_ = nullptr;
};

} // namespace GraphicsLab