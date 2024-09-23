#pragma once

#include "geometry/geometry.hpp"
#include "vkl/core/vkl_device.hpp"
#include "vkl_mesh_types.hpp"

#include "vkl_scene_tree.hpp"

#include <numbers>

namespace SceneTree {

template <typename T> class VklNodeMesh {};

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

// template <> class VklNodeMesh<BezierCurve2D> {
//   public:
//     BezierCurve2D *curve_;
//     VklDevice &device_;
//
//     using render_type = VklCurveMesh2D;
//     using control_points_render_type = VklPointCloudMesh2D;
//     using extreme_point_render_type = VklPointCloud2D;
//     using derivative_bound_render_type = VklCurveMesh2D;
//
//     std::unique_ptr<render_type> curveMesh;
//     std::unique_ptr<control_points_render_type> controlPointsMesh;
//     std::unique_ptr<extreme_point_render_type> extremePointMesh;
//     std::unique_ptr<derivative_bound_render_type> derivativeBoundMesh;
//
//     VklNodeMesh(VklDevice &device, BezierCurve2D *curve) : device_(device), curve_(curve) {
//         createControlPointsMesh();
//
//         if (curve->control_point_vec2.size() >= 2)
//             createMesh();
//     }
//
//     void reallocateMesh() {
//         // @todo: reallocate mesh via updated parametric data
//     }
//
//   private:
//     extreme_point_render_type::BuilderFromImmediateData createExtremePointMeshBuilder() {
//         auto ex_points = curve_->compute_extreme_points();
//         extreme_point_render_type::BuilderFromImmediateData builder;
//         for (auto ep : ex_points) {
//             extreme_point_render_type::vertex_type vertex;
//             vertex.position = ep;
//             vertex.color = {0.0, 1.0, 0.0};
//             builder.vertices.push_back(vertex);
//         }
//         return builder;
//     }
//
//     void createExtremePointMesh() {
//         auto builder = createExtremePointMeshBuilder();
//         if (builder.vertices.empty()) {
//             extremePointMesh = nullptr;
//             return;
//         }
//         extremePointMesh = std::make_unique<extreme_point_render_type>(device_, builder);
//     }
//
//     derivative_bound_render_type::Builder createDerivativeBoundMeshBuilder() {
//         auto start_pos = curve_->control_point_vec2.front();
//         auto end_pos = curve_->control_point_vec2.back();
//
//         float eps = 1e-3;
//
//         derivative_bound_render_type::Builder builder;
//         if (glm::length(end_pos - start_pos) < eps) {
//             // the boundary is a circle
//             auto center = (start_pos + end_pos) / 2.0f;
//             auto bound = curve_->derivative_bound;
//
//             int n = 100;
//
//             for (int i = 0; i <= n; i++) {
//                 float theta = 2.0 * std::numbers::pi / n * i;
//                 derivative_bound_render_type::vertex_type vertex;
//                 vertex.position.x = center.x + bound / 2.0f * std::cos(theta);
//                 vertex.position.y = center.y + bound / 2.0f * std::sin(theta);
//                 vertex.color = {0.0, 1.0, 0.0};
//                 builder.vertices.push_back(vertex);
//             }
//         } else {
//             // the boundary is an ellipse
//             float a = curve_->derivative_bound / 2;
//             float c = glm::length(end_pos - start_pos) / 2;
//             float b = std::sqrt(a * a - c * c);
//
//             int n = 100;
//
//             glm::vec2 center = (start_pos + end_pos) / 2.0f;
//             glm::vec2 x_axis = glm::normalize((end_pos - start_pos) / 2.0f);
//             glm::vec2 y_axis = {x_axis.y, -x_axis.x};
//
//             for (int i = 0; i <= n; i++) {
//                 float theta = 2.0 * std::numbers::pi / n * i;
//                 derivative_bound_render_type::vertex_type vertex;
//                 float x_coord = a * std::cos(theta);
//                 float y_coord = b * std::sin(theta);
//
//                 vertex.position = center + x_axis * x_coord + y_axis * y_coord;
//                 vertex.color = {0.0, 1.0, 0.0};
//
//                 builder.vertices.push_back(vertex);
//             }
//         }
//
//         return builder;
//     }
//
//     void createDerivativeBoundMesh() {
//         auto builder = createDerivativeBoundMeshBuilder();
//
//         derivativeBoundMesh = std::make_unique<derivative_bound_render_type>(device_, builder);
//     }
//
//     void createControlPointsMesh() {
//         control_points_render_type::Builder builder;
//         for (auto cp : curve_->control_point_vec2) {
//             control_points_render_type::vertex_type vertex;
//             vertex.position = cp;
//             builder.vertices.push_back(vertex);
//         }
//         controlPointsMesh = std::make_unique<control_points_render_type>(device_, builder);
//     }
//
//     void createMesh() {
//         render_type::Builder builder;
//         constexpr int n = 100;
//         double param_delta = 1.0 / n;
//
//         for (int i = 0; i <= n; i++) {
//             render_type::vertex_type vertex;
//             auto position = curve_->evaluate(param_delta * i);
//             vertex.position = position;
//             vertex.color = {0.0, 1.0, 0.0};
//             builder.vertices.push_back(vertex);
//         }
//
//         curveMesh = std::make_unique<render_type>(device_, builder);
//     }
// };
//
// template <> class VklNodeMesh<TensorProductBezierSurface> {
//   public:
//     TensorProductBezierSurface *surface_;
//     VklDevice &device_;
//
//     using boundary_render_type = VklCurveMesh3D;
//     using parameter_render_type = VklCurveMesh2D;
//
//     std::vector<std::unique_ptr<boundary_render_type>> boundary_3d;
//     std::vector<std::unique_ptr<parameter_render_type>> boundary_2d;
//
//     VklNodeMesh(VklDevice &device, TensorProductBezierSurface *surf) : device_(device), surface_(surf) {
//         createBoundaryModels();
//         createParameterBoundaryModels();
//     }
//
//   private:
//     void createBoundaryModels() {
//         constexpr int n = 100;
//         double param_delta = 1.0 / n;
//
//         for (auto &boundary : surface_->boundary_curves) {
//             boundary_render_type::Builder builder;
//             for (int i = 0; i <= n; i++) {
//                 boundary_render_type::vertex_type vertex;
//                 auto position = surface_->evaluate(boundary->evaluate(param_delta * i));
//                 vertex.position = position;
//                 builder.vertices.push_back(vertex);
//             }
//             boundary_3d.push_back(std::move(std::make_unique<boundary_render_type>(device_, builder)));
//         }
//     }
//
//     void createParameterBoundaryModels() {
//         constexpr int n = 100;
//         double param_delta = 1.0 / n;
//         for (auto &boundary : surface_->boundary_curves) {
//             parameter_render_type::Builder builder;
//             for (int i = 0; i <= n; i++) {
//                 parameter_render_type::vertex_type vertex;
//                 auto position = boundary->evaluate(param_delta * i);
//                 vertex.position = position;
//                 vertex.color = {1.0, 0.0, 0.0};
//                 builder.vertices.push_back(vertex);
//             }
//             boundary_2d.push_back(std::move(std::make_unique<parameter_render_type>(device_, builder)));
//         }
//     };
// };

template <VklVertexType VertexType, VklIndexType IndexType> class VklNodeMesh<MeshGeometry<VertexType, IndexType>> {
    SceneTree::GeometryNode<MeshGeometry<VertexType, IndexType>> *node_;

    VklDevice &device_;

    void createMesh() {
        typename render_type::Builder builder;

        builder.vertices = node_->data.vertices;
        builder.indices = node_->data.indices;

        if (node_->material_index.has_value()) {
            auto material = node_->scene->materials[node_->material_index.value()];
            std::ranges::copy(material.textures, std::back_inserter(builder.textures));
        }

        mesh = std::make_unique<render_type>(device_, builder);
    }

  public:
    using render_type = VklMesh<VertexType, IndexType>;
    std::unique_ptr<render_type> mesh;

    void recreateMeshes() {
        createMesh();
    }

    VklNodeMesh(VklDevice &device, decltype(node_) node) : device_(device), node_(node) {
        createMesh();
    }
};
} // namespace SceneTree