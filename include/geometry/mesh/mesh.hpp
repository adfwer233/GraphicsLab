#pragma once

#include <vector>

#include "geometry/vertex/null_index.hpp"

#include "../vertex/vertex2d.hpp"
#include "../vertex/vertex3d.hpp"

template <typename VertexType, typename IndicesType> class MeshModelTemplate {
  public:
    struct IsRenderableGeometry {};

    using vertex_type = VertexType;
    using indices_type = IndicesType;

    std::vector<vertex_type> vertices;
    std::vector<IndicesType> indices;
};

template <typename VertexType, typename IndicesType> class MeshGeometry {
public:
    using vertex_type = VertexType;
    using indices_type = IndicesType;

    std::vector<vertex_type> vertices;
    std::vector<IndicesType> indices;
};

using Mesh2D = MeshGeometry<Vertex2D, LineIndex>;
using Mesh3D = MeshGeometry<Vertex3D, TriangleIndex>;
using PointCloud3D = MeshGeometry<Vertex3D, NullIndex>;
using PointCloud2D = MeshGeometry<Vertex2D, NullIndex>;