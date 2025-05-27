#pragma once

#include "vkl/system/render_system/simple_render_system.hpp"

struct PoincareDiskPipelineModifier {
    static void modifyPipeline(PipelineConfigInfo &configInfo) {
        configInfo.inputAssemblyInfo.topology = VkPrimitiveTopology::VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
    }
};

struct PoincareDiskRenderSystemPushConstantData {
    glm::vec2 a, b, c, d;

    static VkShaderStageFlags getStageFlags() {
        return VK_SHADER_STAGE_VERTEX_BIT;
    };
};

using PoincareDiskRenderSystemPushConstantList = VklPushConstantInfoList<PoincareDiskRenderSystemPushConstantData>;

template <uint32_t Subpass = 0>
using PoincareDiskRenderSystem =
    SimpleRenderSystem<Subpass, PoincareDiskRenderSystemPushConstantList, PoincareDiskPipelineModifier>;