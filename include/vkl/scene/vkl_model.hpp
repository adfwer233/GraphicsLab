#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <variant>

#include "glm/glm.hpp"

#include "vkl/core/vkl_buffer.hpp"
#include "vkl/core/vkl_descriptor.hpp"
#include "vkl/core/vkl_device.hpp"
#include "vkl/core/vkl_texture.hpp"
#include "vkl/utils/vkl_box.hpp"

#include "vkl/templates/vkl_concept.hpp"

#include "geometry/mesh/mesh.hpp"
#include "geometry/renderable_geometry.hpp"
#include "geometry/surface/tensor_product_bezier.hpp"
#include "geometry/vertex/null_index.hpp"

template <typename T>
concept VklBoxType = requires(T t) {
    t.min_position;
    t.max_position;
};

template <typename T>
concept VklIndexType = requires {
    { T::vertexCount } -> std::convertible_to<size_t>;
};

template <VklVertexType VertexType, VklIndexType IndexType = TriangleIndex, VklBoxType BoxType = VklBox3D>
class VklModelTemplate {

  public:
    typedef VertexType vertex_type;

    typedef IndexType index_type;

    struct BuilderFromImmediateData {
        std::vector<VertexType> vertices{};
        std::vector<IndexType> indices{};
        std::vector<std::string> texturePaths{};
    };

    std::vector<VklTexture *> textures_;

    std::map<VklDescriptorSetLayoutKey, std::vector<std::unique_ptr<VklBuffer>>> uniformBuffers;
    std::vector<VkDescriptorSet> descriptorSets;

    std::map<VklDescriptorSetLayoutKey, std::vector<VkDescriptorSet>> descriptorSetsMap;

    std::unique_ptr<VklDescriptorSetLayout> descriptorSetLayout_;

    BoxType box; /** Axis aligned bounding box, has not transformed */

  private:
    VklDevice &device_;

    std::vector<std::unique_ptr<VklBuffer>> vertexBuffer_;
    std::vector<std::unique_ptr<VklBuffer>> indexBuffer_;

    uint32_t vertexCount_;
    uint32_t indexCount_;
    bool hasIndexBuffer = false;

    /**
     * @brief create vertex buffer (including staging buffer)
     *
     * staging buffer: belong to cpu, used to upload data
     * vertex buffer: only visible to gpu, used in real graphics pipeline
     *
     * Firstly, we create staging buffer, write data to it. Then create vertex buffer and copy data from the staging
     * buffer to the vertex buffer.
     *
     * @param vertices
     */
    void createVertexBuffers(const std::vector<VertexType> &vertices);

    /**
     * @brief create vertex buffer (including staging buffer)
     *
     * staging buffer: belong to cpu, used to upload data
     * vertex buffer: only visible to gpu, used in real graphics pipeline
     *
     * Firstly, we create staging buffer, write data to it. Then create vertex buffer and copy data from the staging
     * buffer to the vertex buffer.
     *
     * @param vertices
     */
    void createIndexBuffers(const std::vector<IndexType> &indices);

    /**
     * @brief create texture image
     * @param texturePath
     */
    void createTextureImage(const std::string &texturePath);

  public:
    VklModelTemplate(VklDevice &device, BuilderFromImmediateData builder);

    VklModelTemplate(VklDevice &device, MeshModelTemplate<VertexType, IndexType> geometry);

    ~VklModelTemplate();

    VklModelTemplate(const VklModelTemplate &) = delete;
    VklModelTemplate &operator=(const VklModelTemplate &) = delete;

    std::unique_ptr<MeshModelTemplate<VertexType, IndexType>> geometry;

    META_GET_REGISTERED_TYPES(RenderableGeometryTag)::to<std::variant> underlyingGeometry;

    [[nodiscard]] int get_triangle_num() const;

    void allocDescriptorSets(VklDescriptorSetLayout &setLayout, VklDescriptorPool &pool);

    void bind(VkCommandBuffer commandBuffer);

    void draw(VkCommandBuffer commandBuffer);

    void addTextureFromImage(VkImage image);

    int materialIndex = 0;

    VkBuffer getVertexBuffer(size_t index) {
        return vertexBuffer_[index]->getBuffer();
    };

    void updateVertexBuffer();

    void reallocateVertexBuffer();

    friend class RayPicker;
    friend class VklBVH;
};

using VklModel = VklModelTemplate<Vertex3D>;
using VklPointCloud2D = VklModelTemplate<Vertex2D, NullIndex, VklBox2D>;

#include "vkl_model.hpp.impl"