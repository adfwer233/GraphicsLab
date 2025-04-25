#pragma once

#include "simple_render_system.hpp"

struct WireFramePipelineModifier {
    static void modifyPipeline(PipelineConfigInfo &configInfo) {
        configInfo.rasterizationInfo.polygonMode = VkPolygonMode::VK_POLYGON_MODE_LINE;
    }
};

template <uint32_t Subpass = 0>
using SimpleWireFrameRenderSystem = SimpleRenderSystem<Subpass, SimplePushConstantInfoList, WireFramePipelineModifier>;