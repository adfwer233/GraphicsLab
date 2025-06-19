#pragma once

#define NOMINMAX
#include "vkl/scene_tree/vkl_scene_tree.hpp"

#include <vkl/bvh/vkl_bvh_gpu.hpp>

namespace vkl {
struct SceneTreeBvh {
    struct AABB {
        glm::vec3 min, max;

        AABB() {
            min = glm::vec3(std::numeric_limits<float>::max());
            max = glm::vec3(std::numeric_limits<float>::min());
        }

        AABB(glm::vec3 t_min, glm::vec3 t_max) {
            min = t_min;
            max = t_max;
        }
    };

    struct BVHObject {
        int object_index;
        VklBVHGPUModel::Triangle triangle;
    };

    struct BVHNode {
        AABB box;

        int index = -1;
        int leftNodeIndex = -1;
        int rightNodeIndex = -1;

        std::vector<BVHObject> objects;
    };

  private:
    SceneTree::VklSceneTree &scene_;

    static AABB surroundingBox(AABB box0, AABB box1) {
        return {glm::min(box0.min, box1.min), glm::max(box0.max, box1.max)};
    }

    static AABB objectBoundingBox(VklBVHGPUModel::Triangle &tri) {
        constexpr float eps = 1e-4;
        return {glm::min(glm::min(tri.v0, tri.v1), tri.v2) - eps, glm::max(glm::max(tri.v0, tri.v1), tri.v2) + eps};
    }

    static AABB objectListBoundingBox(std::vector<BVHObject> &objects) {
        AABB tempBox;
        AABB outputBox;
        bool firstBox = true;

        for (auto &object : objects) {
            tempBox = objectBoundingBox(object.triangle);
            outputBox = firstBox ? tempBox : surroundingBox(outputBox, tempBox);
            firstBox = false;
        }

        return outputBox;
    }

    inline static bool boxCompare(VklBVHGPUModel::Triangle &a, VklBVHGPUModel::Triangle &b, int axis) {
        AABB boxA = objectBoundingBox(a);
        AABB boxB = objectBoundingBox(b);

        return boxA.min[axis] < boxB.min[axis];
    }

    static bool boxXCompare(BVHObject a, BVHObject b) {
        return boxCompare(a.triangle, b.triangle, 0);
    }

    static bool boxYCompare(BVHObject a, BVHObject b) {
        return boxCompare(a.triangle, b.triangle, 1);
    }

    static bool boxZCompare(BVHObject a, BVHObject b) {
        return boxCompare(a.triangle, b.triangle, 2);
    }

  public:
    std::vector<BVHObject> objects;
    std::vector<VklBVHGPUModel::Light> lights;
    std::vector<VklBVHGPUModel::Triangle> triangles;

    std::vector<SceneTree::MaterialData> materials;

    explicit SceneTreeBvh(SceneTree::VklSceneTree &scene) : scene_(scene) {
    }

    ~SceneTreeBvh() {
    }

    SceneTreeBvh(const VklBVH &) = delete;
    SceneTreeBvh &operator=(const VklBVH &) = delete;

    std::vector<VklBVHGPUModel::BVHNode> createGPUBVHTree() {
        // create bvh objects
        this->objects.clear();

        using RenderableTypeList =
            MetaProgramming::TypeList<Mesh3D, GraphicsLab::Geometry::Sphere, GraphicsLab::Geometry::Torus>;

        glm::vec3 filp(1.0, -1.0, 1.0);
        MetaProgramming::ForEachType(RenderableTypeList{}, [&]<typename T>() {
            auto mesh3d_buffer = SceneTree::VklNodeMeshBuffer<T>::instance();
            for (auto [mesh3d_nodes, trans] : scene_.traverse_geometry_nodes_with_trans<T>()) {
                Mesh3D *mesh = nullptr;
                if constexpr (std::is_same_v<T, GraphicsLab::Geometry::Sphere> or
                              std::is_same_v<T, GraphicsLab::Geometry::Torus>) {
                    mesh = mesh3d_nodes->data.mesh.get();
                } else if constexpr (std::is_same_v<T, Mesh3D>) {
                    mesh = &mesh3d_nodes->data;
                }

                for (size_t k = 0; k < mesh->indices.size(); k++) {
                    auto tri_indices = mesh->indices[k];
                    BVHObject bvhObject;
                    bvhObject.object_index = objects.size();
                    int material_index = mesh3d_nodes->material_index.has_value() ? mesh3d_nodes->material_index.value() : 0;
                    bvhObject.triangle = VklBVHGPUModel::Triangle{
                        trans * glm::vec4(mesh->vertices[tri_indices.i].position * filp, 1.0f),
                        trans * glm::vec4(mesh->vertices[tri_indices.j].position * filp, 1.0f),
                        trans * glm::vec4(mesh->vertices[tri_indices.k].position * filp, 1.0f), static_cast<uint32_t>(material_index)};
                    objects.push_back(bvhObject);
                }
            }
        });

        BVHObject bvhObject;
        bvhObject.object_index = objects.size();
        bvhObject.triangle = VklBVHGPUModel::Triangle{glm::vec4(0.0, 15.0, 5.0, 1.0f), glm::vec4(5.0, 15.0, -5.0, 1.0f),
                                                      glm::vec4(5.0, 15.0, 5.0, 1.0f), static_cast<uint32_t>(1)};
        objects.push_back(bvhObject);

        std::vector<BVHNode> intermediate;
        int nodeCounter = 0;
        std::stack<BVHNode> nodeStack;

        BVHNode root;
        root.index = nodeCounter;
        root.objects = objects;
        nodeCounter++;
        nodeStack.push(root);

        while (!nodeStack.empty()) {
            BVHNode currentNode = nodeStack.top();
            nodeStack.pop();

            currentNode.box = objectListBoundingBox(currentNode.objects);

            int axis = rand() % 3;

            auto comparator = (axis == 0) ? boxXCompare : (axis == 1) ? boxYCompare : boxZCompare;

            if (currentNode.objects.empty())
                continue;

            std::ranges::sort(currentNode.objects, comparator);

            int n = currentNode.objects.size();
            if (n <= 1) {
                intermediate.push_back(currentNode);
                continue;
            } else {
                auto mid = n >> 1;

                BVHNode leftNode, rightNode;

                leftNode.index = nodeCounter;
                nodeCounter++;
                std::copy(currentNode.objects.begin(), currentNode.objects.begin() + mid,
                          std::back_inserter(leftNode.objects));
                nodeStack.push(leftNode);

                rightNode.index = nodeCounter;
                nodeCounter++;
                std::copy(currentNode.objects.begin() + mid, currentNode.objects.end(),
                          std::back_inserter(rightNode.objects));
                nodeStack.push(rightNode);

                currentNode.leftNodeIndex = leftNode.index;
                currentNode.rightNodeIndex = rightNode.index;

                intermediate.push_back(currentNode);
            }
        }

        std::ranges::sort(intermediate, [](BVHNode &a, BVHNode &b) { return a.index < b.index; });

        std::vector<VklBVHGPUModel::BVHNode> output;
        output.reserve(intermediate.size());
        for (auto &node : intermediate) {
            VklBVHGPUModel::BVHNode gpuNode{};
            gpuNode.leftNodeIndex = node.leftNodeIndex;
            gpuNode.rightNodeIndex = node.rightNodeIndex;
            gpuNode.min = node.box.min;
            gpuNode.max = node.box.max;
            if (node.leftNodeIndex == -1 || node.rightNodeIndex == -1) {
                gpuNode.objectIndex = node.objects[0].object_index;
            } else {
                gpuNode.objectIndex = -1;
            }
            output.push_back(gpuNode);
        }

        triangles.reserve(objects.size());
        for (int i = 0; i < objects.size(); i++) {
            triangles.push_back(objects[i].triangle);
        }

        for (auto material : scene_.material_manager.materials) {
            materials.push_back(material.data);
        }

        for (size_t i = 0; i < triangles.size(); i++) {
            auto t = triangles[i];
            //        if (scene_.materials[triangles[i].materialIndex].type ==
            //        VklBVHGPUModel::MaterialType::LightSource) {
            if (scene_.material_manager.materials[triangles[i].materialIndex].data.materialType ==
                SceneTree::MaterialType::Light) {
                float area = glm::length(glm::cross(t.v1 - t.v0, t.v2 - t.v0)) * 0.5f;
                lights.emplace_back(static_cast<uint32_t>(i), area);
            }
        }

        return output;
    }
};
} // namespace vkl