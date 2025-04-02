#pragma once

#define NOMINMAX
#include "igl/per_vertex_normals.h"

#include "Eigen/Eigen"
#include "geometry/geometry.hpp"
#include "language/reflection/static_reflector.hpp"
#include <string>

template <typename T, typename MeshType, typename... Args>
concept GeoFlowPerVertexExecutor = requires(MeshType *mesh, Args... args) {
    { T::run(mesh, args...) } -> std::same_as<Eigen::MatrixXd>;
};

template <typename T>
concept GeoFlowMeshType = requires(T t) {
    typename T::vertex_type;
    { t.vertices } -> std::same_as<std::vector<typename T::vertex_type> &>;
} and MeshVertexConcept<typename T::vertex_type>;

template <GeoFlowMeshType MeshType, typename Executor, typename... Args>
    requires GeoFlowPerVertexExecutor<Executor, MeshType, Args...>
struct GeoFlowPerVertexMap {
    MeshType *mesh;
    std::string targetFieldName;

    void perform(Args... args) {
        Eigen::MatrixXd result = Executor::run(mesh, args...);
        int n = result.rows();

        using vertex_type = typename MeshType::vertex_type;

        for (int i = 0; i < n; i++) {
            glm::vec3 vec{result(i, 0), result(i, 1), result(i, 2)};
            StaticReflect::set_property(mesh->vertices[i], targetFieldName, vec);
        }
    }
};

struct NormalVector {
    static Eigen::MatrixXd run(Mesh3D *mesh) {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        Eigen::MatrixXd K;

        int n = mesh->vertices.size();

        V.resize(n, 3);
        for (int i = 0; i < n; i++) {
            V(i, 0) = mesh->vertices[i].position.x;
            V(i, 1) = -mesh->vertices[i].position.y;
            V(i, 2) = mesh->vertices[i].position.z;
        }

        int nf = mesh->indices.size();
        F.resize(nf, 3);
        for (int i = 0; i < nf; i++) {
            F(i, 0) = mesh->indices[i].i;
            F(i, 1) = mesh->indices[i].j;
            F(i, 2) = mesh->indices[i].k;
        }

        igl::per_vertex_normals(V, F, K);

        return K;
    }
};

template <GeoFlowMeshType MeshType> struct SetColor {
    static Eigen::MatrixXd run(MeshType *mesh, glm::vec3 color) {
        int n = mesh->vertices.size();
        Eigen::MatrixXd V = Eigen::MatrixXd::Zero(n, 3);
        for (int i = 0; i < n; i++) {
            V(i, 0) = color.x;
            V(i, 1) = color.y;
            V(i, 2) = color.z;
        }
        return V;
    }
};

using test = GeoFlowPerVertexMap<Mesh3D, NormalVector>;
using test2 = GeoFlowPerVertexMap<Mesh3D, SetColor<Mesh3D>, glm::vec3>;