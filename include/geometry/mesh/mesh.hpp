#pragma once

#include <vector>

#include "geometry/vertex/null_index.hpp"

#include "../vertex/vertex2d.hpp"
#include "../vertex/vertex3d.hpp"

struct AABB3D {
    glm::vec3 min_pos, max_pos;
};

struct AABB2D {
    glm::vec2 min_pos, max_pos;
};

template <size_t dim> struct AABBType;

template <> struct AABBType<2> {
    using type = AABB2D;
};

template <> struct AABBType<3> {
    using type = AABB3D;
};

template <typename VertexType, typename IndicesType> class MeshModelTemplate {
  public:
    struct IsRenderableGeometry {};

    using vertex_type = VertexType;
    using indices_type = IndicesType;

    std::vector<vertex_type> vertices;
    std::vector<IndicesType> indices;
};

template <typename T>
concept MeshVertexConcept = requires { typename T::IsStaticReflected; };

template <typename T>
concept MeshGeometryTrait = requires { typename T::MeshGeometryTrait; };

template <typename VertexType, typename IndicesType, size_t Dimension = 3> class MeshGeometry {
  public:
    struct MeshGeometryTrait {};
    using vertex_type = VertexType;
    using indices_type = IndicesType;
    using box_type = typename AABBType<Dimension>::type;

    std::vector<vertex_type> vertices;
    std::vector<IndicesType> indices;

    std::optional<box_type> box = std::nullopt;

    box_type getMeshBox() {
        if (box.has_value()) {
            return box.value();
        } else {
            if constexpr (Dimension == 3) {
                auto min_pos = glm::vec3(999999.0);
                auto max_pos = glm::vec3(-999999.0);

                for (size_t i = 0; i < vertices.size(); i++) {
                    min_pos.x = std::min(min_pos.x, vertices[i].position.x);
                    min_pos.y = std::min(min_pos.y, vertices[i].position.y);
                    min_pos.z = std::min(min_pos.z, vertices[i].position.z);

                    max_pos.x = std::max(max_pos.x, vertices[i].position.x);
                    max_pos.y = std::max(max_pos.y, vertices[i].position.y);
                    max_pos.z = std::max(max_pos.z, vertices[i].position.z);
                }

                box = {min_pos, max_pos};

                return box.value();
            } else {
                auto min_pos = glm::vec2(999999.0);
                auto max_pos = glm::vec2(-999999.0);

                for (size_t i = 0; i < vertices.size(); i++) {
                    min_pos.x = std::min(min_pos.x, vertices[i].position.x);
                    min_pos.y = std::min(min_pos.y, vertices[i].position.y);

                    max_pos.x = std::max(max_pos.x, vertices[i].position.x);
                    max_pos.y = std::max(max_pos.y, vertices[i].position.y);
                }

                box = {min_pos, max_pos};

                return box.value();
            }
        }
    }
};

using Mesh2D = MeshGeometry<Vertex2D, TriangleIndex, 2>;
using Mesh3D = MeshGeometry<Vertex3D, TriangleIndex, 3>;
using CurveMesh2D = MeshGeometry<Vertex2D, LineIndex, 2>;
using CurveMesh3D = MeshGeometry<Vertex3D, LineIndex, 3>;
using Wire3D = MeshGeometry<Vertex3DRaw, LineIndex, 3>;
using PointCloud3D = MeshGeometry<Vertex3D, NullIndex, 3>;
using PointCloud2D = MeshGeometry<Vertex2D, NullIndex, 2>;
using DirectionalField3D = MeshGeometry<DirectionalFieldVertex3D, NullIndex, 3>;