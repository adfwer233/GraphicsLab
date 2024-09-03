#pragma once

#include "geometry/surface/surface.hpp"
#include "meta_programming/type_list.hpp"
#include "vkl/scene/vkl_model.hpp"

namespace SceneTree {
enum class NodeType {
    GeometryNode,
    LightSourceNode,
    CameraNode
};

struct PointLightSource {};
struct AreaLightSource {};

using GeometryTypes = MetaProgramming::TypeList<VklModel, TensorProductBezierSurface>;
using LightTypes = MetaProgramming::TypeList<PointLightSource, AreaLightSource>;

template <typename T>
concept SupportedGeometryType = MetaProgramming::TypeListFunctions::IsAnyOf<GeometryTypes, T>::value;

template <typename T>
concept SupportedLightTypes = MetaProgramming::TypeListFunctions::IsAnyOf<LightTypes, T>::value;

template <typename T>
concept SupportedNodeType = SupportedGeometryType<T> || SupportedLightTypes<T>;

template <SupportedGeometryType GeometryType> consteval size_t geometry_type_index() {
    return MetaProgramming::TypeListFunctions::IndexOf<GeometryTypes, GeometryType>::value;
}

template <SupportedLightTypes LightType> consteval size_t light_type_index() {
    return MetaProgramming::TypeListFunctions::IndexOf<LightTypes, LightType>::value;
}

struct TreeNode {
    std::vector<TreeNode *> children;

    template <SupportedNodeType NodeType> void addTreeNode(NodeType *node) {
        children.push_back(reinterpret_cast<TreeNode *>(node));
    }
};

template <SupportedGeometryType GeometryType> struct GeometryNode : public TreeNode {
    const NodeType nodeType = NodeType::GeometryNode;

    GeometryType data;
};

template <SupportedLightTypes LightTypes> struct LightNode : public TreeNode {
    const NodeType nodeType = NodeType::LightSourceNode;
};

struct CameraNode : public TreeNode {
    const NodeType nodeType = NodeType::CameraNode;
};
} // namespace SceneTree

struct VklSceneTree {
    SceneTree::TreeNode *root;
};