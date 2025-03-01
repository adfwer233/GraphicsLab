#pragma once

#include "simple_render_system.hpp"

struct PointCloud3DPipelineModifier {
    static void modifyPipeline(PipelineConfigInfo &configInfo) {
        configInfo.inputAssemblyInfo.topology = VkPrimitiveTopology::VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
//        configInfo.depthStencilInfo.depthTestEnable = VK_FALSE;
//        configInfo.colorBlendAttachment.blendEnable = VK_TRUE;
    }
};

using PointCloud3DRenderSystem = SimpleRenderSystem<0, SimplePushConstantInfoList, PointCloud3DPipelineModifier>;