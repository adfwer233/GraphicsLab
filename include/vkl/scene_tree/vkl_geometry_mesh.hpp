#pragma once

#include "vkl/core/vkl_device.hpp"
#include "vkl_mesh_types.hpp"

#include "vkl_scene_tree.hpp"

#include <geometry/parametric/parametric_space.hpp>
#include <geometry/parametric/sphere.hpp>
#include <geometry/parametric/traits.hpp>
#include <numbers>

namespace SceneTree {

template <typename T> class VklNodeMesh {
    using type = T;
};

template <typename T> class VklNodeMeshBuffer {
  public:
    VklNodeMesh<T> *getGeometryModel(VklDevice &device, GeometryNode<T> *ptr) {
        if (map_.contains(ptr))
            return map_[ptr];
        else {
            map_[ptr] = new VklNodeMesh<T>(device, ptr);
            return map_[ptr];
        }
    }

    static VklNodeMeshBuffer<T> *instance() {
        if (instance_ == nullptr) {
            instance_ = new VklNodeMeshBuffer<T>();
        }
        return instance_;
    }

    ~VklNodeMeshBuffer() {
        for (auto &[k, v] : map_) {
            delete v;
        }
    }

    static void free_instance() {
        if (instance_ == nullptr)
            return;
        for (auto [geometry, mesh] : instance_->map_) {
            delete mesh;
        }
    }

  private:
    std::map<GeometryNode<T> *, VklNodeMesh<T> *> map_;
    static inline VklNodeMeshBuffer<T> *instance_ = nullptr;
    // VklGeometryModelBuffer<T>() = default;
};

template <VklVertexType VertexType, VklIndexType IndexType, size_t dimension>
class VklNodeMesh<MeshGeometry<VertexType, IndexType, dimension>> {
    using box_type = std::conditional_t<dimension == 3, VklBox3D, VklBox2D>;
    SceneTree::GeometryNode<MeshGeometry<VertexType, IndexType, dimension>> *node_;

    VklDevice &device_;

    void createMesh() {
        typename render_type::Builder builder;

        builder.vertices = node_->data.vertices;
        builder.indices = node_->data.indices;

        if (node_->material_index.has_value()) {
            auto material = node_->scene->material_manager.materials[node_->material_index.value()];
            for (auto texture : material.textures) {
                auto vkl_texture = node_->scene->texture_manager.get_texture(texture);
                builder.textures.emplace_back(vkl_texture);
            }
            // std::ranges::copy(material.textures, std::back_inserter(builder.textures));
        } else {
            int x = 0;
        }

        mesh = std::make_unique<render_type>(device_, builder);
    }

  public:
    using render_type = VklMesh<VertexType, IndexType, box_type>;
    std::unique_ptr<render_type> mesh;

    void recreateMeshes() {
        createMesh();
    }

    VklNodeMesh(VklDevice &device, decltype(node_) node) : device_(device), node_(node) {
        createMesh();
    }
};

template <GraphicsLab::Geometry::IsParametricSurface T> class VklNodeMesh<T> {
    SceneTree::GeometryNode<T> *node_;

    VklDevice &device_;

    void createMesh() {
        typename render_type::Builder builder;

        builder.vertices = node_->data.mesh->vertices;
        builder.indices = node_->data.mesh->indices;

        mesh = std::make_unique<render_type>(device_, builder);
    }

  public:
    using render_type = VklMesh<Vertex3D, TriangleIndex>;
    std::unique_ptr<render_type> mesh;

    void recreateMeshes() {
        createMesh();
    }

    VklNodeMesh(VklDevice &device, decltype(node_) node) : node_(node), device_(device) {
        createMesh();
    }
};

template <GraphicsLab::Geometry::IsParametricCurve2D T> class VklNodeMesh<T> {
    SceneTree::GeometryNode<T> *node_;

    VklDevice &device_;

    void createMesh() {
        typename render_type::Builder builder;

        builder.vertices = node_->data.mesh->vertices;
        builder.indices = node_->data.mesh->indices;

        mesh = std::make_unique<render_type>(device_, builder);
    }

  public:
    using render_type = VklCurveMesh2D;
    std::unique_ptr<render_type> mesh;

    void recreateMeshes() {
        createMesh();
    }

    VklNodeMesh(VklDevice &device, decltype(node_) node) : node_(node), device_(device) {
        createMesh();
    }
};

template <GraphicsLab::Geometry::IsParametricCurve3D T> class VklNodeMesh<T> {
    SceneTree::GeometryNode<T> *node_;

    VklDevice &device_;

    void createMesh() {
        typename render_type::Builder builder;

        builder.vertices = node_->data.mesh->vertices;
        builder.indices = node_->data.mesh->indices;

        mesh = std::make_unique<render_type>(device_, builder);
    }

  public:
    using render_type = VklCurveMesh3D;
    std::unique_ptr<render_type> mesh;

    void recreateMeshes() {
        createMesh();
    }

    VklNodeMesh(VklDevice &device, decltype(node_) node) : node_(node), device_(device) {
        createMesh();
    }
};

} // namespace SceneTree