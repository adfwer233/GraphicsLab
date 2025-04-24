#pragma once

#include "language/meta_programming/type_list.hpp"

#include "language/reflection/reflectors.hpp"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include "vkl_camera.hpp"
#include "vkl_texture_manager.hpp"
// #include "vkl_material.hpp"

#include "glm/gtc/quaternion.hpp"

#include <geometry/parametric/parametric_surface.hpp>
#include <vkl/bvh/vkl_bvh_gpu.hpp>

#include "graphics_lab/geo_flow/geo_flow.hpp"

#include <geometry/parametric/bezier_curve_2d.hpp>
#include <geometry/parametric/torus.hpp>

struct Material;

namespace SceneTree {
enum class NodeType {
    InternalNode,
    GeometryNode,
    LightSourceNode,
    CameraNode
};

struct PointLightSource {};
struct AreaLightSource {};

using GeometryTypes = MetaProgramming::TypeList<Mesh3D, CurveMesh3D, Wire3D, DirectionalField3D, PointCloud3D,
                                                PointCloud2D, GraphicsLab::Geometry::BezierCurve2D>;
using LightTypes = MetaProgramming::TypeList<PointLightSource, AreaLightSource>;

template <typename T>
concept SupportedGeometryType = MetaProgramming::TypeListFunctions::IsAnyOf<GeometryTypes, T>::value ||
                                requires { typename T::SceneTreeGeometryTypeTrait; };

template <typename T>
concept SupportedLightTypes = MetaProgramming::TypeListFunctions::IsAnyOf<LightTypes, T>::value;

template <SupportedGeometryType T> struct GeometryNode;
template <SupportedLightTypes LightTypes> struct LightNode;

template <typename T>
concept SupportedNodeType = MetaProgramming::TypeListFunctions::IsAnyOf<GeometryTypes::monad<GeometryNode>, T>::value ||
                            MetaProgramming::TypeListFunctions::IsAnyOf<LightTypes::monad<LightNode>, T>::value;

template <SupportedGeometryType GeometryType> consteval size_t geometry_type_index() {
    return MetaProgramming::TypeListFunctions::IndexOf<GeometryTypes, GeometryType>::value;
}

template <SupportedLightTypes LightType> consteval size_t light_type_index() {
    return MetaProgramming::TypeListFunctions::IndexOf<LightTypes, LightType>::value;
}

struct VklSceneTree;

struct GraphicsTransformation : Reflectable {
    glm::vec3 translation = glm::vec3(0.0f);
    glm::vec3 scaling = glm::vec3(1.0f);
    glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    [[nodiscard]] glm::mat4 getTransformation() const {
        glm::mat4 model(1.0f);
        model = glm::translate(model, translation); // Apply translation
        model *= quatToMat4(rotation);              // Apply rotation
        model = glm::scale(model, scaling);         // Apply scaling

        return model;
    }

    ReflectDataType reflect() override {
        return {{"translation", TypeErasedValue(&translation)},
                {"scaling", TypeErasedValue(&scaling)},
                {"rotation", TypeErasedValue(&rotation)}};
    }

    glm::mat4 quatToMat4(const glm::quat &q) const {
        glm::mat4 result(1.0f); // Initialize with identity matrix

        float x2 = q.x + q.x;
        float y2 = q.y + q.y;
        float z2 = q.z + q.z;

        float xx = q.x * x2;
        float yy = q.y * y2;
        float zz = q.z * z2;

        float xy = q.x * y2;
        float xz = q.x * z2;
        float yz = q.y * z2;

        float wx = q.w * x2;
        float wy = q.w * y2;
        float wz = q.w * z2;

        result[0][0] = 1.0f - (yy + zz);
        result[0][1] = xy + wz;
        result[0][2] = xz - wy;

        result[1][0] = xy - wz;
        result[1][1] = 1.0f - (xx + zz);
        result[1][2] = yz + wx;

        result[2][0] = xz + wy;
        result[2][1] = yz - wx;
        result[2][2] = 1.0f - (xx + yy);

        return result;
    }
};

struct TreeNode : public Reflectable {
    std::vector<std::unique_ptr<TreeNode>> children;
    std::string name;

    VklSceneTree *scene = nullptr;

    template <SupportedNodeType NodeType> void addTreeNode(std::unique_ptr<NodeType> node) {
        children.push_back(std::move(node));
        for (auto &child : children) {
            child->scene = scene;
        }
    }

    ReflectDataType reflect() override {
        return {{"name", TypeErasedValue(&name)}};
    }
    virtual NodeType type() = 0;
};

struct InternalNode : public TreeNode {
    ReflectDataType reflect() override {
        auto result = TreeNode::reflect();
        result.emplace("transformation", TypeErasedValue(&transformation));
        return result;
    }

    NodeType type() override {
        return NodeType::InternalNode;
    }

    GraphicsTransformation transformation;
};

template <SupportedGeometryType GeometryType> struct GeometryNode : public TreeNode {
    NodeType type() override {
        return NodeType::GeometryNode;
    }

    ReflectDataType reflect() override {
        auto result = TreeNode::reflect();
        result.emplace("transformation", TypeErasedValue(&transformation));
        result.emplace("visible", TypeErasedValue(&visible));
        result.emplace("Apply Transformation", TypeErasedValue(&GeometryNode::applyTransformation, this));
        result.emplace("rotateX", TypeErasedValue(&GeometryNode::rotateX, this));
        result.emplace("rotateY", TypeErasedValue(&GeometryNode::rotateY, this));
        result.emplace("rotateZ", TypeErasedValue(&GeometryNode::rotateZ, this));

        result.emplace("updateColor",
                       TypeErasedValue(&GeometryNode::updateColor, this,
                                       std::tuple<glm::vec3>(glm::vec3(1.0f, 1.0f, 1.0f)), {"New Color"}));
        result.emplace("testReflectionFunction",
                       TypeErasedValue(&GeometryNode::testReflectionFunction, this, {1, 2, 3}, {"x", "y", "z"}));
        return result;
    }

    GeometryNode()
        requires std::default_initializable<GeometryType>
    = default;
    explicit GeometryNode(GeometryType &&geometry_data) : data(std::move(geometry_data)) {
    }

    GeometryType data;
    bool visible = true;
    std::optional<size_t> material_index = std::nullopt;
    GraphicsTransformation transformation;

    // sync objects
    bool updated = false;
    bool boxUpdated = false;

    void updateColor(glm::vec3 color) {
        if constexpr (MeshGeometryTrait<GeometryType>) {
            if constexpr (StaticReflect::HasField<typename GeometryType::vertex_type, "color">()) {
                GeoFlowPerVertexMap<GeometryType, SetColor<GeometryType>, glm::vec3> set_color;
                set_color.mesh = &this->data;
                set_color.targetFieldName = "color";
                set_color.perform(color);
            }
        } else if constexpr (GraphicsLab::Geometry::ParamSurfaceTrait<GeometryType>) {
            GeoFlowPerVertexMap<Mesh3D, SetColor<Mesh3D>, glm::vec3> set_color;
            set_color.mesh = this->data.mesh.get();
            set_color.targetFieldName = "color";
            set_color.perform(color);
        }

        updated = true;
    }

    void testReflectionFunction(int x, int y, int z) {
        spdlog::info("x: {}, y: {}, z: {}", x, y, z);
    }

    void applyTransformation() {
        if constexpr (std::is_same_v<GeometryType, Mesh3D>) {
            spdlog::info("apply transformation to mesh 3d");

            auto trans = transformation.getTransformation();

            auto &mesh = static_cast<Mesh3D &>(data);

            for (auto &vert : mesh.vertices) {
                vert.position = trans * glm::vec4(vert.position, 1.0f);
            }
            transformation.translation = glm::vec3(0.0f);
            transformation.scaling = glm::vec3(1.0f);
            transformation.rotation = glm::quat(0.0f, 0.0f, 1.0f, 0.0f);

            updated = true;
            boxUpdated = true;

            mesh.box.reset();
        }
    }

    void rotateX() {
        if constexpr (std::is_same_v<GeometryType, Mesh3D>) {
            const glm::quat rotation = glm::angleAxis(glm::radians(45.0f), glm::normalize(glm::vec3{1.0f, 0.0f, 0.0f}));
            transformation.rotation = rotation * transformation.rotation;
        }
    }

    void rotateY() {
        if constexpr (std::is_same_v<GeometryType, Mesh3D>) {
            const glm::quat rotation = glm::angleAxis(glm::radians(45.0f), glm::normalize(glm::vec3{0.0f, 1.0f, 0.0f}));
            transformation.rotation = rotation * transformation.rotation;
        }
    }

    void rotateZ() {
        if constexpr (std::is_same_v<GeometryType, Mesh3D>) {
            const glm::quat rotation = glm::angleAxis(glm::radians(45.0f), glm::normalize(glm::vec3{0.0f, 0.0f, 1.0f}));
            transformation.rotation = rotation * transformation.rotation;
        }
    }
};

template <SupportedLightTypes LightTypes> struct LightNode : public TreeNode {
    NodeType type() override {
        return NodeType::LightSourceNode;
    }
};

struct CameraNode : public TreeNode {
    NodeType type() override {
        return NodeType::CameraNode;
    }

    Camera camera;

    explicit CameraNode(Camera t_camera) : camera(t_camera) {
    }

    void move() {
        camera.position += glm::vec3{0, 0, 10};
    }

    ReflectDataType reflect() override {
        auto base_reflect = TreeNode::reflect();
        base_reflect.emplace("position", TypeErasedValue(&camera.position));
        base_reflect.emplace("move", TypeErasedValue(&CameraNode::move, this));
        return base_reflect;
    }
};

class AssimpImporter {
  private:
    VklDevice &device_;
    const std::string path;
    std::string directory;

  public:
    AssimpImporter(VklDevice &device, const std::string &filePath) : device_(device), path(filePath) {
        Assimp::Importer importer;
        auto scene = importer.ReadFile(filePath, aiProcess_Triangulate | aiProcess_FlipUVs);
        this->directory = path.substr(0, path.find_last_of('/'));
        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            throw std::runtime_error("Failed to load model: " + std::string(importer.GetErrorString()));
        }
    }

    std::unique_ptr<TreeNode> importScene() {
        Assimp::Importer importer;
        auto scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs |
                                                 aiProcess_JoinIdenticalVertices | aiProcess_GenNormals);

        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            throw std::runtime_error("Failed to load model: " + std::string(importer.GetErrorString()));
        }

        return processNode(scene->mRootNode, scene);
    }

    // auto importMaterial() {
    //     Assimp::Importer importer;
    //     auto scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs);
    //
    //     if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
    //         throw std::runtime_error("Failed to load model: " + std::string(importer.GetErrorString()));
    //     }
    //
    //     std::vector<Material> materials;
    //     //
    //     // for (int i = 0; i < scene->mNumMaterials; i++) {
    //     //     auto material = scene->mMaterials[i];
    //     //
    //     //     Material mat;
    //     //
    //     //     material->Get(AI_MATKEY_COLOR_DIFFUSE, mat.diffuse);
    //     //     material->Get(AI_MATKEY_COLOR_SPECULAR, mat.specular);
    //     //     material->Get(AI_MATKEY_COLOR_AMBIENT, mat.ambient);
    //     //     material->Get(AI_MATKEY_COLOR_EMISSIVE, mat.emissive);
    //     //     material->Get(AI_MATKEY_SHININESS, mat.shininess);
    //     //
    //     //     int count = material->GetTextureCount(aiTextureType::aiTextureType_DIFFUSE);
    //     //
    //     //     for (int j = 0; j < material->GetTextureCount(aiTextureType::aiTextureType_DIFFUSE); j++) {
    //     //         aiString str;
    //     //         material->GetTexture(aiTextureType::aiTextureType_DIFFUSE, j, &str);
    //     //
    //     //         mat.textures.push_back(
    //     //             createTextureImage(std::format("{}/{}", this->directory, std::string(str.C_Str()))));
    //     //     }
    //     //
    //     //     materials.push_back(mat);
    //     // }
    //
    //     return materials;
    // }

  private:
    VklTexture *createTextureImage(const std::string &texturePath) {
        int texWidth, texHeight, texChannels;
        stbi_uc *pixels = stbi_load(texturePath.c_str(), &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);
        VkDeviceSize imageSize = texWidth * texHeight * texChannels;

        if (!pixels) {
            throw std::runtime_error("failed to load texture image!");
        }

        if (texChannels == 3) {
            throw std::runtime_error("unsupported texture info");
        }

        VklBuffer stagingBuffer{device_, imageSize, 1, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};
        stagingBuffer.map();
        stagingBuffer.writeToBuffer((void *)pixels);
        stagingBuffer.unmap();

        auto texture = new VklTexture(device_, texWidth, texHeight, texChannels);

        device_.transitionImageLayout(texture->image_, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_UNDEFINED,
                                      VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        device_.copyBufferToImage(stagingBuffer.getBuffer(), texture->image_, static_cast<uint32_t>(texWidth),
                                  static_cast<uint32_t>(texHeight), 1);
        device_.transitionImageLayout(texture->image_, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

        stbi_image_free(pixels);

        return texture;
    }

    std::unique_ptr<TreeNode> processNode(aiNode *node, const aiScene *scene) {
        auto internalNode = std::make_unique<InternalNode>();

        // Process each mesh located at the current node
        for (unsigned int i = 0; i < node->mNumMeshes; i++) {
            aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
            auto geometryNode = convertMeshToGeometryNode(mesh);
            geometryNode->name = mesh->mName.C_Str();
            internalNode->addTreeNode<GeometryNode<Mesh3D>>(std::move(geometryNode));
        }

        // Process each child node recursively
        for (unsigned int i = 0; i < node->mNumChildren; i++) {
            auto childNode = processNode(node->mChildren[i], scene);
            childNode->scene = internalNode->scene;
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

        spdlog::info("normal {}", mesh->HasNormals());

        for (auto i = 0; i < mesh->mNumFaces; i++) {
            aiFace face = mesh->mFaces[i];
            mesh_converted.indices.emplace_back(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
        }

        auto mesh_node = std::make_unique<GeometryNode<Mesh3D>>();
        mesh_node->data = std::move(mesh_converted);

        mesh_node->material_index = mesh->mMaterialIndex;

        return std::move(mesh_node);
    }
};

struct VklSceneTree {
    VklDevice &device_;
    std::unique_ptr<SceneTree::TreeNode> root;
    std::vector<VklBVHGPUModel::Material> materials;

    CameraNode *active_camera = nullptr;
    TreeNode *activeNode = nullptr;

    std::function<void()> sceneUpdateCallBack;

    std::mutex sceneTreeMutex;

    vkl::TextureManager texture_manager;

    void sceneUpdated() {
        if (sceneUpdateCallBack) {
            sceneUpdateCallBack();
        }
    }

    explicit VklSceneTree(VklDevice &device) : device_(device), texture_manager(device) {
        root = std::make_unique<SceneTree::InternalNode>();
        spdlog::info("Scene Tree Created");

        VklBVHGPUModel::Material gray{VklBVHGPUModel::MaterialType::Lambertian, 0.0, 0.5, 1.0,
                                      glm::vec3(0.77f, 0.77f, 0.80f)};
        VklBVHGPUModel::Material red{VklBVHGPUModel::MaterialType::Lambertian, 0.0, 0.5, 1.0,
                                     glm::vec3(0.8f, 0.0f, 0.0f)};
        VklBVHGPUModel::Material green{VklBVHGPUModel::MaterialType::Lambertian, 0.0, 0.5, 1.0,
                                       glm::vec3(0.0f, 0.8f, 0.0f)};
        VklBVHGPUModel::Material whiteLight{VklBVHGPUModel::MaterialType::LightSource, 0.0, 0.0, 1.0,
                                            glm::vec3(2.0f, 2.0f, 2.0f)};
        VklBVHGPUModel::Material metal{VklBVHGPUModel::MaterialType::Metal, 0.5, 0.0, 1.0, glm::vec3(1.0f, 0.9f, 0.0f)};
        VklBVHGPUModel::Material glass{VklBVHGPUModel::MaterialType::Glass, 0.0, 0.0, 1.0, glm::vec3(1.0f, 1.0f, 1.0f)};

        materials.push_back(gray);
        materials.push_back(red);
        materials.push_back(green);
        materials.push_back(whiteLight);
        materials.push_back(metal);
        materials.push_back(glass);
    }

    template <SupportedGeometryType GeometryType>
    void addGeometryNode(GeometryType &&Geometry, std::optional<std::string> name = std::nullopt) {
        auto geometry_node = std::make_unique<GeometryNode<GeometryType>>(std::forward<GeometryType>(Geometry));
        if (name.has_value()) {
            geometry_node->name = name.value();
        }
        root->children.push_back(std::move(geometry_node));
    }

    void addCameraNode(const std::string &name, Camera camera) {
        if (root != nullptr) {
            auto camera_node = std::make_unique<CameraNode>(camera);
            camera_node->name = name;
            if (active_camera == nullptr) {
                active_camera = camera_node.get();
            }
            root->children.push_back(std::move(camera_node));
        }
    }

    void importFromPath(const std::string &path) {
        AssimpImporter assimpImporter(device_, path);
        root = assimpImporter.importScene();
        // materials = assimpImporter.importMaterial();
        set_scene_to_nodes(root.get());
        active_camera = nullptr;
    }

    void cleanSceneTree() {
        active_camera = nullptr;
        activeNode = nullptr;

        materials.clear();
        root.reset();
    }

    template <SupportedGeometryType GeometryType> GeometryNode<GeometryType> *get_geometry_node(std::string_view name) {
        for (auto node : traverse_geometry_nodes<GeometryType>()) {
            if (node->name == name) {
                return node;
            }
        }
        return nullptr;
    }

    template <SupportedGeometryType GeometryType> Generator<GeometryNode<GeometryType> *> traverse_geometry_nodes() {
        for (auto node : traverse_geometry_nodes_internal<GeometryType>(root.get())) {
            co_yield node;
        }
    }

    template <SupportedGeometryType GeometryType>
    Generator<std::pair<GeometryNode<GeometryType> *, glm::mat4>> traverse_geometry_nodes_with_trans() {
        for (auto [node, trans] :
             traverse_geometry_nodes_with_trans_internal<GeometryType>(root.get(), glm::mat4(1.0f))) {
            co_yield {node, trans};
        }
    }

    ~VklSceneTree() {
        spdlog::info("Scene Tree Destructed");

        // for (auto &mat : materials) {
        //     for (auto tex : mat.textures) {
        //         delete tex;
        //     }
        // }
    }

  private:
    template <SupportedGeometryType GeometryType>
    Generator<GeometryNode<GeometryType> *> traverse_geometry_nodes_internal(TreeNode *node) {
        if (auto geometryNode = dynamic_cast<GeometryNode<GeometryType> *>(node)) {
            co_yield geometryNode;
        }

        for (auto &child : node->children) {
            for (auto geometryNode : traverse_geometry_nodes_internal<GeometryType>(child.get())) {
                co_yield geometryNode;
            }
        }
    }

    template <SupportedGeometryType GeometryType>
    Generator<std::pair<GeometryNode<GeometryType> *, glm::mat4>> traverse_geometry_nodes_with_trans_internal(
        TreeNode *node, glm::mat4 currentTransformation) {
        glm::mat4 trans = currentTransformation;

        if (auto internal_node = dynamic_cast<InternalNode *>(node)) {
            trans = currentTransformation * internal_node->transformation.getTransformation();
        }

        if (auto geometryNode = dynamic_cast<GeometryNode<GeometryType> *>(node)) {
            co_yield {geometryNode, currentTransformation * geometryNode->transformation.getTransformation()};
        }

        for (auto &child : node->children) {
            for (auto [geometryNode, nodeTransf] :
                 traverse_geometry_nodes_with_trans_internal<GeometryType>(child.get(), trans)) {
                co_yield {geometryNode, nodeTransf};
            }
        }
    }

    void set_scene_to_nodes(TreeNode *node) {
        node->scene = this;
        for (auto &child : node->children) {
            set_scene_to_nodes(child.get());
        }
    }
};

} // namespace SceneTree
