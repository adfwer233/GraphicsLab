#pragma once

#include "Eigen/Eigen"
#include "geometry/geometry.hpp"
#include "igl/per_vertex_normals.h"
#include "language/reflection/static_reflector.hpp"
#include <string>

template <typename T, typename MeshType>
concept GeoFlowPerVertexExecutor = requires(MeshType *mesh) {
    { T::run(mesh) } -> std::same_as<Eigen::MatrixXd>;
};

template <typename T>
concept GeoFlowMeshType = requires(T t) {
    typename T::vertex_type;
    { t.vertices } -> std::same_as<std::vector<typename T::vertex_type> &>;
} and MeshVertexConcept<typename T::vertex_type>;

template <GeoFlowMeshType MeshType, GeoFlowPerVertexExecutor<MeshType> Executor> struct GeoFlowPerVertexMap {
    MeshType *mesh;
    std::string targetFieldName;

    void perform() {
        Eigen::MatrixXd result = Executor::run(mesh);
        int n = result.rows();

        using vertex_type = MeshType::vertex_type;

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

using test = GeoFlowPerVertexMap<Mesh3D, NormalVector>;