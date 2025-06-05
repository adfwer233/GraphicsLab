#include "vkl/core/vkl_compute_pipeline.hpp"

#include "spdlog/spdlog.h"

#include <fstream>

void VklComputePipeline::createComputePipeline(const std::string &compFilePath,
                                               const ComputePipelineConfigInfo &configInfo) {
    auto computeShaderCode = readFile(compFilePath);
    createShaderModule(computeShaderCode, &computeShaderModule_);

    VkPipelineShaderStageCreateInfo computeShaderStageInfo{};
    computeShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    computeShaderStageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    computeShaderStageInfo.module = computeShaderModule_;
    computeShaderStageInfo.pName = "main";

    VkComputePipelineCreateInfo pipelineCreateInfo{};
    pipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipelineCreateInfo.layout = configInfo.computePipelineLayout;
    pipelineCreateInfo.stage = computeShaderStageInfo;

    VkResult result = vkCreateComputePipelines(device_.device(), VK_NULL_HANDLE, 1, &pipelineCreateInfo, nullptr,
                                 &computePipeline_);

    if (result == VK_ERROR_OUT_OF_HOST_MEMORY) {
        spdlog::error("failed to create compute pipeline, out of host memory, compute shader path is {}", compFilePath);
        throw std::runtime_error("failed to create compute pipeline, out of host memory");
    } else if (result == VK_ERROR_OUT_OF_DEVICE_MEMORY) {
        spdlog::error("failed to create compute pipeline, out of device memory, compute shader path is {}", compFilePath);
        throw std::runtime_error("failed to create compute pipeline, out of device memory");
    } else if (result != VK_SUCCESS) {
        spdlog::error("failed to create compute pipeline, compute shader path is {}, result = {:X}", compFilePath, size_t(result));
        throw std::runtime_error("failed to create compute pipeline");
    }
}

VklComputePipeline::VklComputePipeline(VklDevice &device, const std::string &compFilePath,
                                       const ComputePipelineConfigInfo &configInfo)
    : VklPipeline(device) {
    createComputePipeline(compFilePath, configInfo);
}

VklComputePipeline::~VklComputePipeline() {
    vkDestroyShaderModule(device_.device(), computeShaderModule_, nullptr);
    vkDestroyPipeline(device_.device(), computePipeline_, nullptr);
}
