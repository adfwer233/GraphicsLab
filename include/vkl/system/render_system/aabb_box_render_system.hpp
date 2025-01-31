#pragma once

#include "simple_render_system.hpp"
#include "glm/glm.hpp"

struct Box3DRenderSystemModifier{
    static constexpr const int poolSize = 20000;
    static void modifyPipeline(PipelineConfigInfo &configInfo) {
        configInfo.inputAssemblyInfo.topology = VkPrimitiveTopology::VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
    }
};

struct Box3DRenderSystemPushConstantData {
    glm::mat4 view_proj;
    glm::vec4 min_pos, max_pos;
    static VkShaderStageFlags getStageFlags() {
        return VK_SHADER_STAGE_GEOMETRY_BIT;
    }
};

using Box3DRenderSystem = SimpleRenderSystem<0, VklPushConstantInfoList<Box3DRenderSystemPushConstantData>, Box3DRenderSystemModifier>;