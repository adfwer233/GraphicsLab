#define NOMINMAX
#pragma once

#include "vkl_bvh_gpu.hpp"

#include <numeric>

class VklScene;

class VklBVH {
  public:
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
    VklScene &scene_;

    static AABB surroundingBox(AABB box0, AABB box1);

    static AABB objectBoundingBox(VklBVHGPUModel::Triangle &tri);

    static AABB objectListBoundingBox(std::vector<BVHObject> &objects);

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

    explicit VklBVH(VklScene &scene) : scene_(scene) {
    }
    ~VklBVH() {
    }

    VklBVH(const VklBVH &) = delete;
    VklBVH &operator=(const VklBVH &) = delete;

    std::vector<VklBVHGPUModel::BVHNode> createGPUBVHTree();
};