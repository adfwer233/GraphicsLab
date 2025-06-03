#pragma once

#include "vkl/system/render_system/simple_render_system.hpp"

struct PoincareDiskPointCloudPipelineModifier {
    static void modifyPipeline(PipelineConfigInfo &configInfo) {
        configInfo.inputAssemblyInfo.topology = VkPrimitiveTopology::VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
        configInfo.depthStencilInfo.depthTestEnable = VK_FALSE;
        configInfo.colorBlendAttachment.blendEnable = VK_TRUE;
    }
};

struct PoincareDiskPointCloudRenderSystemPushConstantData {
    glm::vec2 a, b, c, d;

    static VkShaderStageFlags getStageFlags() {
        return VK_SHADER_STAGE_VERTEX_BIT;
    };
};

using PoincareDiskPointCloudRenderSystemPushConstantList = VklPushConstantInfoList<PoincareDiskPointCloudRenderSystemPushConstantData>;

template <uint32_t Subpass = 0>
using PoincareDiskPointCloudRenderSystem =
    SimpleRenderSystem<Subpass, PoincareDiskPointCloudRenderSystemPushConstantList, PoincareDiskPointCloudPipelineModifier>;