#pragma once

#include "geometry/surface/surface.hpp"
#include "meta_programming/type_list.hpp"
#include "vkl/scene/vkl_model.hpp"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

namespace SceneTree {
enum class NodeType {
    InternalNode,
    GeometryNode,
    LightSourceNode,
    CameraNode
};

struct PointLightSource {};
struct AreaLightSource {};

using GeometryTypes = MetaProgramming::TypeList<Mesh3D, TensorProductBezierSurface>;
using LightTypes = MetaProgramming::TypeList<PointLightSource, AreaLightSource>;

template <typename T>
concept SupportedGeometryType = MetaProgramming::TypeListFunctions::IsAnyOf<GeometryTypes, T>::value;

template <typename T>
concept SupportedLightTypes = MetaProgramming::TypeListFunctions::IsAnyOf<LightTypes, T>::value;

template<SupportedGeometryType T> struct GeometryNode;
template <SupportedLightTypes LightTypes> struct LightNode;

template <typename T>
concept SupportedNodeType = MetaProgramming::TypeListFunctions::IsAnyOf<GeometryTypes::monad<GeometryNode>, T>::value || MetaProgramming::TypeListFunctions::IsAnyOf<LightTypes::monad<LightNode>, T>::value;

template <SupportedGeometryType GeometryType> consteval size_t geometry_type_index() {
    return MetaProgramming::TypeListFunctions::IndexOf<GeometryTypes, GeometryType>::value;
}

template <SupportedLightTypes LightType> consteval size_t light_type_index() {
    return MetaProgramming::TypeListFunctions::IndexOf<LightTypes, LightType>::value;
}

struct TreeNode {
    std::vector<std::unique_ptr<TreeNode>> children;
    std::string name;

    template <SupportedNodeType NodeType> void addTreeNode(std::unique_ptr<NodeType> node) {
        children.push_back(std::move(node));
    }

    virtual NodeType type() = 0;
};

struct InternalNode: public TreeNode {
    NodeType type() override { return NodeType::InternalNode; }
};

template <SupportedGeometryType GeometryType> struct GeometryNode : public TreeNode {
    NodeType type() override { return NodeType::GeometryNode; }

    GeometryType data;

    explicit GeometryNode(GeometryType &&t_data) {
        data = std::move(t_data);
    }

};

template <SupportedLightTypes LightTypes> struct LightNode : public TreeNode {
    NodeType type() override { return NodeType::LightSourceNode; }
};

struct CameraNode : public TreeNode {
    NodeType type() override { return NodeType::CameraNode; }
};

class AssimpImporter {
  public:
    std::unique_ptr<TreeNode> importScene(const std::string& filePath) {
        Assimp::Importer importer;
        const aiScene* scene = importer.ReadFile(filePath, aiProcess_Triangulate | aiProcess_FlipUVs);

        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            throw std::runtime_error("Failed to load model: " + std::string(importer.GetErrorString()));
        }

        return processNode(scene->mRootNode, scene);
    }

  private:
    std::unique_ptr<TreeNode> processNode(aiNode* node, const aiScene* scene) {
        auto internalNode = std::make_unique<InternalNode>();

        // Process each mesh located at the current node
        for (unsigned int i = 0; i < node->mNumMeshes; i++) {
            aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
            auto geometryNode = convertMeshToGeometryNode(mesh);
            geometryNode->name = mesh->mName.C_Str();
            internalNode->addTreeNode<GeometryNode<Mesh3D>>(std::move(geometryNode));
        }

        // Process each child node recursively
        for (unsigned int i = 0; i < node->mNumChildren; i++) {
            auto childNode = processNode(node->mChildren[i], scene);
            internalNode->children.push_back(std::move(childNode));
        }

        internalNode->name = node->mName.C_Str();

        return internalNode;
    }

    static std::unique_ptr<GeometryNode<Mesh3D>> convertMeshToGeometryNode(aiMesh *mesh) {
        Mesh3D mesh_converted;

        for (auto i = 0; i < mesh->mNumVertices; i++) {
            decltype(mesh_converted.vertices)::value_type vertex;
            vertex.position = {mesh->mVertices[i].x, -mesh->mVertices[i].y, mesh->mVertices[i].z};
            if (mesh->HasNormals())
                vertex.normal = {mesh->mNormals[i].x, -mesh->mNormals[i].y, mesh->mNormals[i].z};
            if (mesh->mTextureCoords[0])
                vertex.uv = {mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y};
            mesh_converted.vertices.push_back(vertex);
        }

        for (auto i = 0; i < mesh->mNumFaces; i++) {
            aiFace face = mesh->mFaces[i];
            mesh_converted.indices.emplace_back(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
        }

        // @todo: add material importer

        auto mesh_mode = std::make_unique<GeometryNode<Mesh3D>>(std::move(mesh_converted));

        return std::move(mesh_mode);
    }
};

struct VklSceneTree {
    std::unique_ptr<SceneTree::TreeNode> root;

    VklSceneTree() {
        root = std::make_unique<SceneTree::InternalNode>();
    }

    template<SupportedGeometryType GeometryType>
    void addGeometryNode(GeometryType &&Geometry) {
        auto geometry_node = std::make_unique<GeometryNode<GeometryType>>(Geometry);
        root->children.push_back(std::move(geometry_node));
    }

    void importFromPath(const std::string& path) {
        AssimpImporter assimpImporter;
        root = assimpImporter.importScene(path);
    }

    template<SupportedGeometryType GeometryType>
    Generator<GeometryNode<GeometryType>*> traverse_geometry_nodes() {
        for (auto node: traverse_geometry_nodes_internal<GeometryType>(root.get())) {
            co_yield node;
        }
    }
private:
    template <SupportedGeometryType GeometryType>
    Generator<GeometryNode<GeometryType>*> traverse_geometry_nodes_internal(TreeNode* node) {
        if (auto geometryNode = dynamic_cast<GeometryNode<GeometryType>*>(node)) {
            co_yield geometryNode;
        }

        for (auto& child : node->children) {
            for (auto geometryNode : traverse_geometry_nodes_internal<GeometryType>(child.get())) {
                co_yield geometryNode;
            }
        }
    }
};

} // namespace SceneTree
