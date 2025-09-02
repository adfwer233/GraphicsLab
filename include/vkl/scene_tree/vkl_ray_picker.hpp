#pragma once

#include "glm/glm.hpp"
#include "vkl_scene_tree.hpp"

namespace SceneTree {

class Ray {
  public:
    glm::vec3 base;
    glm::vec3 dir;

    Ray() {
    }

    Ray(const glm::vec3 &t_base, const glm::vec3 &t_dir) : base(t_base), dir(t_dir) {
    }

    glm::vec3 at(float t) const {
        return base + t * dir;
    };

    struct ray_triangle_intersection_result {
        bool hit;
        float param, u, v, w;
    };

    auto ray_triangle_intersection(const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2) const {

        ray_triangle_intersection_result result{false, 0, 0, 0, 0};

        // compute the normal of triangle
        constexpr float eps = 1e-8;

        auto normal = glm::cross(v1 - v0, v2 - v0);
        float area2 = glm::length(normal);

        if (area2 < eps) {
            return result;
        }
        normal = normal / area2;

        auto n_dot_dir = glm::dot(normal, this->dir);
        if (std::fabs(n_dot_dir) < eps) {
            return result;
        }

        float d = -1.0f * glm::dot(normal, v0);

        float t = -1.0f * (glm::dot(normal, this->base) + d) / n_dot_dir;

        if (t < eps)
            return result;

        auto p = this->at(t);

        auto get_param = [&](const glm::vec3 &r, const glm::vec3 &s) -> float {
            auto edge = r - s;
            auto tmp = p - s;
            auto c = glm::cross(edge, tmp);
            auto area = glm::length(c);
            auto dot = glm::dot(normal, c);
            if (glm::dot(normal, c) < 0)
                return -1;
            return area / area2;
        };

        result.w = get_param(v1, v0);
        result.u = get_param(v2, v1);
        result.v = get_param(v0, v2);
        result.param = t;

        if (result.u < 0 or result.v < 0 or result.w < 0)
            return result;

        result.hit = true;
        return result;
    }
};

class RayPicker {
  public:
    RayPicker(VklSceneTree &sceneTree, Ray &ray) : sceneTree_(sceneTree), ray_(ray) {
    }

    struct RayPickingResult {
        TreeNode *hitGeometryNode;
        float param;
        float u, v, w;
    };

    std::optional<RayPickingResult> trace() {

        std::map<float, RayPickingResult> param_result_map;

        for (auto [node, trans] : sceneTree_.traverse_geometry_nodes_with_trans<Mesh3D>()) {
            for (auto face : node->data.indices) {
                auto int_res =
                    ray_.ray_triangle_intersection(trans * glm::vec4(node->data.vertices[face.i].position, 1.0f),
                                                   trans * glm::vec4(node->data.vertices[face.j].position, 1.0f),
                                                   trans * glm::vec4(node->data.vertices[face.k].position, 1.0f));

                if (int_res.hit) {
                    param_result_map[int_res.param] = {.hitGeometryNode = node,
                                                       .param = int_res.param,
                                                       .u = int_res.u,
                                                       .v = int_res.v,
                                                       .w = int_res.w};
                }
            }
        }

        MetaProgramming::ForEachType(GraphicsLab::Geometry::ParamSurfaceTypeList{}, [&]<typename T>() {
            for (auto [param_node, trans] : sceneTree_.traverse_geometry_nodes_with_trans<T>()) {
                Mesh3D *mesh = param_node->data.mesh.get();
                for (auto face : mesh->indices) {
                    auto int_res =
                        ray_.ray_triangle_intersection(trans * glm::vec4(mesh->vertices[face.i].position, 1.0f),
                                                       trans * glm::vec4(mesh->vertices[face.j].position, 1.0f),
                                                       trans * glm::vec4(mesh->vertices[face.k].position, 1.0f));
                    if (int_res.hit) {
                        param_result_map[int_res.param] = {.hitGeometryNode = param_node,
                                                           .param = int_res.param,
                                                           .u = int_res.u,
                                                           .v = int_res.v,
                                                           .w = int_res.w};
                    }
                }
            }
        });

        std::optional<RayPickingResult> result = std::nullopt;

        if (not param_result_map.empty()) {
            auto &[t, res] = *param_result_map.begin();
            result = res;
        }

        return result;
    }

  private:
    VklSceneTree &sceneTree_;
    Ray ray_;
};

} // namespace SceneTree