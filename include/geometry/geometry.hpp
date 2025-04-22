#pragma once

#include "geometry/mesh/mesh.hpp"
#include "geometry/vertex/vertex.hpp"

#include "language/meta_programming/type_list.hpp"

using BuiltinVertexTypes = MetaProgramming::TypeList<Vertex2D, Vertex2DRaw, Vertex3D, Vertex3DRaw>;

template <typename T>
concept ValidGeometryType = requires { typename T::MeshDataType; };

using GeometryTypes = MetaProgramming::TypeList<>;