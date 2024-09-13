#pragma once

#include "vkl_scene_tree.hpp"
#include "glm/glm.hpp"

namespace SceneTree {

class Ray {
public:
    glm::vec3 base;
    glm::vec3 dir;

    Ray() {}

    Ray(const glm::vec3 &t_base, const glm::vec3 &t_dir) : base(t_base), dir(t_dir) {}

    glm::vec3 at(float t) const {
        return base + t * dir;
    };

    auto ray_triangle_intersection(const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2) const {

        struct {
            bool hit = false;
            float param = 0, u = 0, v = 0, w = 0;
        } result;

        using T = decltype(result);

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

        float u, v, w;

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

        if (u < 0 or v < 0 or w < 0)
            return result;

        return result;
    }
};


class RayPicker {
public:
    RayPicker (VklSceneTree& sceneTree, Ray &ray): sceneTree_(sceneTree) {}

    struct RayPickingResult {
        TreeNode* hitGeometryNode;
        float param;
        float u, v, w;
    };

    std::optional<RayPickingResult> trace() {

        std::map<float, RayPickingResult> param_result_map;
        
        for (auto [node, trans]: sceneTree_.traverse_geometry_nodes_with_trans<Mesh3D>() ) {

        }

        // for (size_t object_index = 0; object_index < scene_.objects.size(); object_index++) {
        //     auto &object = scene_.objects[object_index];
        //
        //     Ray object_ray = ray_;
        //     glm::mat4 model_transformation = object->getModelTransformation();
        //
        //     for (size_t model_index = 0; model_index < object->models.size(); model_index++) {
        //         auto model = object->models[model_index];
        //
        //         for (size_t face_index = 0; face_index < model->geometry->indices.size(); face_index++) {
        //             auto &face = model->geometry->indices[face_index];
        //
        //             auto [flag, t, u, v, w] = object_ray.ray_triangle_intersection(
        //                     model_transformation * glm::vec4(model->geometry->vertices[face.i].position, 1.0f),
        //                     model_transformation * glm::vec4(model->geometry->vertices[face.j].position, 1.0f),
        //                     model_transformation * glm::vec4(model->geometry->vertices[face.k].position, 1.0f));
        //
        //             if (flag) {
        //                 param_result_map[t] = {object_index, model_index, face_index, 0, t, u, v, w};
        //             }
        //         }
        //     }
        // }
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

}