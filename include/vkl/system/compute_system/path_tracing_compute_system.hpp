#pragma once

#include <format>

#include "base_compute_system.hpp"

#include "vkl/bvh/vkl_bvh.hpp"
#include "vkl/core/vkl_image.hpp"

#include "glm/glm.hpp"

struct PathTracingUniformBufferObject {
    alignas(16) glm::vec3 cameraPosition;
    alignas(16) glm::vec3 cameraUp;
    alignas(16) glm::vec3 cameraFront;
    float cameraZoom;
    float rand1;
    float rand2;
    uint32_t currentSample;
    uint32_t numTriangles;
    uint32_t numLights;
};

class PathTracingComputeModel {
  private:
    VklDevice &device_;
    VklScene &scene_;

    std::vector<ComputeDescriptor<VklBuffer>> uniformBufferDescriptors_;
    std::vector<ComputeDescriptor<VklBuffer>> storageBufferDescriptors_;
    std::vector<ComputeDescriptor<VklImage>> imageDescriptors_;

    std::vector<VklBuffer *> uniformBuffers;
    std::vector<VklBuffer *> storageBuffers;
    std::vector<VklImage *> images;

  public:
    static std::string get_comp_shader_path() {
        return std::format("{}/path_tracing_compute_shader.comp.spv", SHADER_DIR);
    }

    PathTracingUniformBufferObject ubo;

    PathTracingComputeModel(VklDevice &device, VklScene &scene) : device_(device), scene_(scene) {
        VklBVH bvh(scene);
        auto bvhTree = bvh.createGPUBVHTree();

        const int n = 1024;
        const int m = 1024;

        ubo.cameraPosition = scene.camera.position;
        ubo.cameraUp = scene.camera.camera_up_axis;
        ubo.cameraFront = scene.camera.camera_front;
        ubo.cameraZoom = scene.camera.zoom;
        ubo.currentSample = 0;
        ubo.numTriangles = bvh.triangles.size();
        ubo.numLights = bvh.lights.size();

        VklBuffer stagingBuffer0{device_, sizeof(PathTracingUniformBufferObject), 1, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};

        stagingBuffer0.map();
        stagingBuffer0.writeToBuffer((void *)&ubo);

        auto uniformBuffer = new VklBuffer(device, sizeof(PathTracingUniformBufferObject), 1,
                                           VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                           VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        device.copyBuffer(stagingBuffer0.getBuffer(), uniformBuffer->getBuffer(),
                          sizeof(PathTracingUniformBufferObject));

        uniformBuffers.push_back(uniformBuffer);
        for (int i = 0; i < uniformBuffers.size(); i++) {
            uniformBufferDescriptors_.push_back({uniformBuffers[i], VK_SHADER_STAGE_COMPUTE_BIT});
        }

        /*
         * create textures
         */
        auto targetTexture = new VklImage(device_, n, m,
                                          VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT |
                                              VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT);

        device_.transitionImageLayout(targetTexture->image_, VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_LAYOUT_UNDEFINED,
                                      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

        auto accumulationTexture = new VklImage(device_, n, m,
                                                VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT |
                                                    VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT);
        device_.transitionImageLayout(accumulationTexture->image_, VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_LAYOUT_UNDEFINED,
                                      VK_IMAGE_LAYOUT_GENERAL);

        images.push_back(targetTexture);
        images.push_back(accumulationTexture);

        for (auto &image : images) {
            imageDescriptors_.push_back({image, VK_SHADER_STAGE_COMPUTE_BIT});
        }

        /*
         * create storage buffers
         */

        // create triangle buffer
        uint32_t trianglesNum = static_cast<uint32_t>(bvh.triangles.size());
        VklBuffer stagingBuffer1{device_, sizeof(VklBVHGPUModel::Triangle), trianglesNum,
                                 VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};

        stagingBuffer1.map();
        stagingBuffer1.writeToBuffer((void *)bvh.triangles.data());

        auto triangleBuffer = new VklBuffer(device, sizeof(VklBVHGPUModel::Triangle), trianglesNum,
                                            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        device.copyBuffer(stagingBuffer1.getBuffer(), triangleBuffer->getBuffer(),
                          sizeof(VklBVHGPUModel::Triangle) * trianglesNum);

        // create material buffer

        uint32_t materialNum = static_cast<uint32_t>(scene.materials.size());
        VklBuffer stagingBuffer2{device_, sizeof(VklBVHGPUModel::Material), materialNum,
                                 VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};

        stagingBuffer2.map();
        stagingBuffer2.writeToBuffer((void *)scene.materials.data());

        auto materialBuffer = new VklBuffer(device, sizeof(VklBVHGPUModel::Material), materialNum,
                                            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        device.copyBuffer(stagingBuffer2.getBuffer(), materialBuffer->getBuffer(),
                          sizeof(VklBVHGPUModel::Material) * materialNum);

        // create aabb buffer

        uint32_t aabbNum = static_cast<uint32_t>(bvhTree.size());
        VklBuffer stagingBuffer3{device, sizeof(VklBVHGPUModel::BVHNode), aabbNum, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};
        stagingBuffer3.map();
        stagingBuffer3.writeToBuffer((void *)bvhTree.data());

        auto aabbBuffer = new VklBuffer(device, sizeof(VklBVHGPUModel::BVHNode), aabbNum,
                                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        device.copyBuffer(stagingBuffer3.getBuffer(), aabbBuffer->getBuffer(),
                          sizeof(VklBVHGPUModel::BVHNode) * aabbNum);

        // create light buffer

        uint32_t lightNum = static_cast<uint32_t>(bvh.lights.size());
        VklBuffer stagingBuffer4{device, sizeof(VklBVHGPUModel::Light), lightNum, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};
        stagingBuffer4.map();
        stagingBuffer4.writeToBuffer((void *)bvh.lights.data());

        auto lightBuffer = new VklBuffer(device, sizeof(VklBVHGPUModel::Light), lightNum,
                                         VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        device.copyBuffer(stagingBuffer4.getBuffer(), lightBuffer->getBuffer(),
                          sizeof(VklBVHGPUModel::Light) * lightNum);

        storageBuffers.push_back(triangleBuffer);
        storageBuffers.push_back(materialBuffer);
        storageBuffers.push_back(aabbBuffer);
        storageBuffers.push_back(lightBuffer);

        for (int i = 0; i < storageBuffers.size(); i++) {
            storageBufferDescriptors_.push_back({storageBuffers[i], VK_SHADER_STAGE_COMPUTE_BIT});
        }
    }

    void resetGPUBVH() {
        VklBVH bvh(scene_);
        auto bvhTree = bvh.createGPUBVHTree();

        const int n = 1024;
        const int m = 1024;

        /*
         * update storage buffers
         */

        // update triangle buffer
        uint32_t trianglesNum = static_cast<uint32_t>(bvh.triangles.size());
        VklBuffer stagingBuffer1{device_, sizeof(VklBVHGPUModel::Triangle), trianglesNum,
                                 VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};

        stagingBuffer1.map();
        stagingBuffer1.writeToBuffer((void *)bvh.triangles.data());

        device_.copyBuffer(stagingBuffer1.getBuffer(), storageBuffers[0]->getBuffer(),
                           sizeof(VklBVHGPUModel::Triangle) * trianglesNum);

        // create material buffer

        uint32_t materialNum = static_cast<uint32_t>(scene_.materials.size());
        VklBuffer stagingBuffer2{device_, sizeof(VklBVHGPUModel::Material), materialNum,
                                 VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};

        stagingBuffer2.map();
        stagingBuffer2.writeToBuffer((void *)scene_.materials.data());

        device_.copyBuffer(stagingBuffer2.getBuffer(), storageBuffers[1]->getBuffer(),
                           sizeof(VklBVHGPUModel::Material) * materialNum);

        // create aabb buffer

        uint32_t aabbNum = static_cast<uint32_t>(bvhTree.size());
        VklBuffer stagingBuffer3{device_, sizeof(VklBVHGPUModel::BVHNode), aabbNum, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};
        stagingBuffer3.map();
        stagingBuffer3.writeToBuffer((void *)bvhTree.data());

        device_.copyBuffer(stagingBuffer3.getBuffer(), storageBuffers[2]->getBuffer(),
                           sizeof(VklBVHGPUModel::BVHNode) * aabbNum);

        // create light buffer

        uint32_t lightNum = static_cast<uint32_t>(bvh.lights.size());
        VklBuffer stagingBuffer4{device_, sizeof(VklBVHGPUModel::Light), lightNum, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};
        stagingBuffer4.map();
        stagingBuffer4.writeToBuffer((void *)bvh.lights.data());

        device_.copyBuffer(stagingBuffer4.getBuffer(), storageBuffers[3]->getBuffer(),
                           sizeof(VklBVHGPUModel::Light) * lightNum);

        ubo.numLights = lightNum;
    }

    ~PathTracingComputeModel() {
        for (auto buffer : uniformBuffers) {
            delete buffer;
        }
        for (auto buffer : storageBuffers) {
            delete buffer;
        }
        for (auto image : images) {
            delete image;
        }
    }

    std::vector<ComputeDescriptor<VklBuffer>> getUniformBufferDescriptors() {
        return uniformBufferDescriptors_;
    }
    std::vector<ComputeDescriptor<VklImage>> getImageDescriptors() {
        return imageDescriptors_;
    }
    std::vector<ComputeDescriptor<VklBuffer>> getStorageDescriptor() {
        return storageBufferDescriptors_;
    }

    std::tuple<int, int, int> getLocalSize() {
        return {32, 32, 1};
    }

    std::tuple<int, int, int> getSize() {
        return {1024, 1024, 1};
    }

    VkImage getTargetTexture() {
        return this->images[0]->image_;
    }

    VkImage getAccumulationTexture() {
        return this->images[1]->image_;
    }

    void recordCommandBuffer(VkCommandBuffer commandBuffer, VkImage targetTexture, VkImage accumulationTexture,
                             VkImage renderOutputImage, uint32_t frameIndex, VklComputePipeline *pipeline_,
                             VkPipelineLayout pipelineLayout, std::vector<VkDescriptorSet> &descriptorSets) {
        VkImageMemoryBarrier read2Gen = VklImageUtils::ReadOnlyToGeneralBarrier(targetTexture);

        vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                             0, nullptr, 0, nullptr, 1, &read2Gen);

        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_->computePipeline_);

        vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1,
                                &descriptorSets[frameIndex], 0, nullptr);

        auto [local_x, local_y, local_z] = getLocalSize();
        auto [x, y, z] = getSize();

        vkCmdDispatch(commandBuffer, x / local_x, y / local_y, z / local_z);

        VkImageMemoryBarrier gen2TranSrc = VklImageUtils::generalToTransferSrcBarrier(targetTexture);

        vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0,
                             nullptr, 0, nullptr, 1, &gen2TranSrc);

        VkImageMemoryBarrier gen2TranDst = VklImageUtils::generalToTransferDstBarrier(accumulationTexture);

        vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0,
                             nullptr, 0, nullptr, 1, &gen2TranDst);

        VkImageMemoryBarrier resultRead2Gen = VklImageUtils::ReadOnlyToDstBarrier(renderOutputImage);
        vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                             0, nullptr, 0, nullptr, 1, &resultRead2Gen);

        VkImageCopy region = VklImageUtils::imageCopyRegion(1024, 1024);
        vkCmdCopyImage(commandBuffer, targetTexture, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, accumulationTexture,
                       VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

        vkCmdCopyImage(commandBuffer, targetTexture, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, renderOutputImage,
                       VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

        VkImageMemoryBarrier resultTranGen2Dst = VklImageUtils::transferDstToReadOnlyBarrier(renderOutputImage);
        vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0,
                             nullptr, 0, nullptr, 1, &resultTranGen2Dst);

        VkImageMemoryBarrier tranDst2Gen = VklImageUtils::transferDstToGeneralBarrier(accumulationTexture);
        vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0,
                             nullptr, 0, nullptr, 1, &tranDst2Gen);

        VkImageMemoryBarrier tranSrc2ReadOnly = VklImageUtils::transferSrcToReadOnlyBarrier(targetTexture);
        vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0,
                             nullptr, 0, nullptr, 1, &tranSrc2ReadOnly);
    }
};

using PathTracingComputeSystem = BaseComputeSystem<PathTracingUniformBufferObject, PathTracingComputeModel>;
