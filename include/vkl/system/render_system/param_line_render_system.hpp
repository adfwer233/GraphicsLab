#pragma once

#include "simple_render_system.hpp"

struct ParamLinePipelineModifier {
    static void modifyPipeline(PipelineConfigInfo &configInfo) {
        configInfo.inputAssemblyInfo.topology = VkPrimitiveTopology::VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
    }
};

struct ParamLineRenderSystemPushConstantData {
    float zoom, shift_x, shift_y;

    static VkShaderStageFlags getStageFlags() {
        return VK_SHADER_STAGE_VERTEX_BIT;
    };
};

using ParamLineRenderSystemPushConstantList = VklPushConstantInfoList<ParamLineRenderSystemPushConstantData>;

template <uint32_t Subpass = 0>
using ParamLineRenderSystem =
    SimpleRenderSystem<Subpass, ParamLineRenderSystemPushConstantList, ParamLinePipelineModifier>;