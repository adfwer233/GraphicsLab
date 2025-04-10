#pragma once

#include "simple_render_system.hpp"

struct PureShaderRenderSystemModifier {
    static constexpr const int poolSize = 20000;
    static void modifyPipeline(PipelineConfigInfo &configInfo) {
        configInfo.inputAssemblyInfo.topology = VkPrimitiveTopology::VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
    }
};

struct PureShaderRenderSystemPushConstantData {
    float zoom, offset_x, offset_y;

    static VkShaderStageFlags getStageFlags() {
        return VK_SHADER_STAGE_GEOMETRY_BIT;
    }
};

using PureShaderRenderSystem = SimpleRenderSystem<0, VklPushConstantInfoList<PureShaderRenderSystemPushConstantData>,
                                                  PureShaderRenderSystemModifier>;