#pragma once

#include "vkl_mesh.hpp"

namespace SceneTree {
using VklCurveMesh3D = VklMesh<Vertex3D, LineIndex, VklBox3D>;
using VklCurveMesh2D = VklMesh<Vertex2D, LineIndex, VklBox2D>;
using VklPointCloudMesh3D = VklMesh<Vertex3D, NullIndex, VklBox3D>;
using VklPointCloudMesh2D = VklMesh<Vertex2D, NullIndex, VklBox2D>;

using VklDirectionalFieldMesh = VklMesh<Vertex3D, NullIndex, VklBox3D>;

using VklMesh3D = VklMesh<Vertex3D, TriangleIndex, VklBox3D>;
using VklMesh2D = VklMesh<Vertex2D, LineIndex, VklBox2D>;
} // namespace SceneTree