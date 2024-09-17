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

namespace SceneTree {

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
class VklMesh {
  public:
    using vertex_type = VertexType;
    using index_type = IndexType;
    using box_type = BoxType;

    std::vector<VklTexture *> textures_;
    std::vector<VkDescriptorSet> descriptorSets_;

    struct Builder {
        std::vector<VertexType> vertices{};
        std::vector<IndexType> indices{};
        std::vector<VklTexture *> textures{};
    };

    box_type meshBox_;

  private:
    VklDevice &device_;

    std::vector<std::unique_ptr<VklBuffer>> vertexBuffer_;
    std::vector<std::unique_ptr<VklBuffer>> indexBuffer_;

    uint32_t vertexCount_;
    uint32_t indexCount_;
    bool hasIndexBuffer = false;

    void createVertexBuffers(const std::vector<VertexType> &vertices);
    void createIndexBuffers(const std::vector<IndexType> &indices);
    void createTextureImage(const std::string &texturePath);

    void createMeshBox(const std::vector<VertexType> &vertices);

  public:
    VklMesh(VklDevice &device, Builder builder);

    ~VklMesh();

    VklMesh(const VklMesh &) = delete;
    VklMesh &operator=(const VklMesh &) = delete;

    void bind(VkCommandBuffer commandBuffer);
    void draw(VkCommandBuffer commandBuffer);

    std::map<VklDescriptorSetLayoutKey, std::vector<std::unique_ptr<VklBuffer>>> uniformBuffers;
    std::map<VklDescriptorSetLayoutKey, std::vector<VkDescriptorSet>> descriptorSetsMap;
    void allocDescriptorSets(VklDescriptorSetLayout &setLayout, VklDescriptorPool &pool);

    box_type getMeshBox() {
        return meshBox_;
    }

    int materialIndex = 0;

    VkBuffer getVertexBuffer(size_t index) {
        return vertexBuffer_[index]->getBuffer();
    };
};

template <VklVertexType VertexType, VklIndexType IndexType, VklBoxType BoxType>
void VklMesh<VertexType, IndexType, BoxType>::createMeshBox(const std::vector<VertexType> &vertices) {
    if constexpr (std::is_same_v<BoxType, VklBox3D>) {
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

        meshBox_.min_position = min_pos;
        meshBox_.max_position = max_pos;
    } else {
        auto min_pos = glm::vec2(999999.0);
        auto max_pos = glm::vec2(-999999.0);

        for (size_t i = 0; i < vertices.size(); i++) {
            min_pos.x = std::min(min_pos.x, vertices[i].position.x);
            min_pos.y = std::min(min_pos.y, vertices[i].position.y);

            max_pos.x = std::max(max_pos.x, vertices[i].position.x);
            max_pos.y = std::max(max_pos.y, vertices[i].position.y);
        }

        meshBox_.min_position = min_pos;
        meshBox_.max_position = max_pos;
    }
}

template <VklVertexType VertexType, VklIndexType IndexType, VklBoxType BoxType>
void VklMesh<VertexType, IndexType, BoxType>::allocDescriptorSets(VklDescriptorSetLayout &setLayout,
                                                                  VklDescriptorPool &pool) {
    auto key = setLayout.descriptorSetLayoutKey;
    if (not descriptorSetsMap.contains(key)) {
        descriptorSetsMap[key].clear();
        descriptorSetsMap[key].resize(VklSwapChain::MAX_FRAMES_IN_FLIGHT);
        if (not key.uniformDescriptors.empty())
            uniformBuffers[key].resize(VklSwapChain::MAX_FRAMES_IN_FLIGHT);

        std::vector<VklDescriptorWriter> writer;
        for (int i = 0; i < VklSwapChain::MAX_FRAMES_IN_FLIGHT; i++) {
            writer.emplace_back(setLayout, pool);
        }
        for (auto &uniformBufferDescriptor : key.uniformDescriptors) {
            for (int i = 0; i < uniformBuffers[key].size(); i++) {
                uniformBuffers[key][i] = std::move(std::make_unique<VklBuffer>(device_, uniformBufferDescriptor.size, 1,
                                                                               VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                                               VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT));
                uniformBuffers[key][i]->map();
                auto bufferInfo = uniformBuffers[key][i]->descriptorInfo();
                writer[i].writeBuffer(uniformBufferDescriptor.binding, &bufferInfo);
            }
        }

        for (int i = 0; auto &sampledImages : key.sampledImageBufferDescriptors) {
            auto imageInfo = textures_[i]->descriptorInfo();
            for (int j = 0; j < VklSwapChain::MAX_FRAMES_IN_FLIGHT; j++) {
                writer[j].writeImage(sampledImages.binding, &imageInfo);
            }
            i++;
        }

        for (int i = 0; i < VklSwapChain::MAX_FRAMES_IN_FLIGHT; i++) {
            writer[i].build(descriptorSetsMap[key][i]);
        }
    }
}

template <VklVertexType VertexType, VklIndexType IndexType, VklBoxType BoxType>
void VklMesh<VertexType, IndexType, BoxType>::draw(VkCommandBuffer commandBuffer) {
    if (hasIndexBuffer) {
        vkCmdDrawIndexed(commandBuffer, indexCount_, 1, 0, 0, 0);
    } else {
        vkCmdDraw(commandBuffer, vertexCount_, 1, 0, 0);
    }
}

template <VklVertexType VertexType, VklIndexType IndexType, VklBoxType BoxType>
void VklMesh<VertexType, IndexType, BoxType>::bind(VkCommandBuffer commandBuffer) {
    VkBuffer buffers[] = {vertexBuffer_[0]->getBuffer()};
    VkDeviceSize offsets[] = {0};

    vkCmdBindVertexBuffers(commandBuffer, 0, 1, buffers, offsets);

    if (hasIndexBuffer) {
        vkCmdBindIndexBuffer(commandBuffer, indexBuffer_[0]->getBuffer(), 0, VK_INDEX_TYPE_UINT32);
    }
}

template <VklVertexType VertexType, VklIndexType IndexType, VklBoxType BoxType>
VklMesh<VertexType, IndexType, BoxType>::~VklMesh() {
}

template <VklVertexType VertexType, VklIndexType IndexType, VklBoxType BoxType>
void VklMesh<VertexType, IndexType, BoxType>::createTextureImage(const std::string &texturePath) {
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

    this->textures_.push_back(texture);

    stbi_image_free(pixels);
}

template <VklVertexType VertexType, VklIndexType IndexType, VklBoxType BoxType>
void VklMesh<VertexType, IndexType, BoxType>::createIndexBuffers(const std::vector<IndexType> &indices) {
    indexCount_ = static_cast<uint32_t>(indices.size()) * IndexType::vertexCount;
    hasIndexBuffer = indexCount_ > 0;

    if (not hasIndexBuffer)
        return;

    uint32_t indexSize = sizeof(uint32_t);
    VkDeviceSize bufferSize = indexSize * indexCount_;

    VklBuffer stagingBuffer{device_, indexSize, indexCount_, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};

    stagingBuffer.map();
    stagingBuffer.writeToBuffer((void *)indices.data());

    for (int i = 0; i < VklSwapChain::MAX_FRAMES_IN_FLIGHT; i++) {
        auto indexBuffer = std::make_unique<VklBuffer>(
            device_, indexSize, indexCount_, VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        device_.copyBuffer(stagingBuffer.getBuffer(), indexBuffer->getBuffer(), bufferSize);

        indexBuffer_.push_back(std::move(indexBuffer));
    }
}

template <VklVertexType VertexType, VklIndexType IndexType, VklBoxType BoxType>
void VklMesh<VertexType, IndexType, BoxType>::createVertexBuffers(const std::vector<VertexType> &vertices) {
    vertexCount_ = static_cast<uint32_t>(vertices.size());
    uint32_t vertexSize = sizeof(typename std::remove_reference<decltype(vertices)>::type::value_type);
    VkDeviceSize bufferSize = vertexSize * vertexCount_;

    VklBuffer stagingBuffer{device_, vertexSize, vertexCount_, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};

    stagingBuffer.map();
    stagingBuffer.writeToBuffer((void *)vertices.data());

    for (int i = 0; i < VklSwapChain::MAX_FRAMES_IN_FLIGHT; i++) {
        auto vertexBuffer = std::make_unique<VklBuffer>(
            device_, vertexSize, vertexCount_,
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        device_.copyBuffer(stagingBuffer.getBuffer(), vertexBuffer->getBuffer(), bufferSize);

        vertexBuffer_.push_back(std::move(vertexBuffer));
    }
}

template <VklVertexType VertexType, VklIndexType IndexType, VklBoxType BoxType>
VklMesh<VertexType, IndexType, BoxType>::VklMesh(VklDevice &device, VklMesh::Builder builder) : device_(device) {
    createVertexBuffers(builder.vertices);
    createIndexBuffers(builder.indices);
    createMeshBox(builder.vertices);
    std::ranges::copy(builder.textures, std::back_inserter(textures_));
}

} // namespace SceneTree