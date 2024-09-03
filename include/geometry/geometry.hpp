#pragma once

#include "geometry/vertex/vertex.hpp"
#include "geometry/mesh/mesh.hpp"
#include "geometry/curve/curve.hpp"
#include "geometry/surface/surface.hpp"

#include "meta_programming/type_list.hpp"

using BuiltinVertexTypes = MetaProgramming::TypeList<Vertex2D, Vertex2DRaw, Vertex3D, Vertex3DRaw>;
using BuiltinGeometryTypes = MetaProgramming::TypeList<Mesh2D, Mesh3D, PointCloud2D, PointCloud3D, TensorProductBezierSurface, BezierCurve2D>;

template<typename T>
concept ValidGeometryType = requires {
    typename T::MeshDataType;
};

using GeometryTypes = MetaProgramming::TypeList<>;