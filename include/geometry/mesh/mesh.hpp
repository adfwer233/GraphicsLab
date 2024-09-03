#pragma once

#include <vector>

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
