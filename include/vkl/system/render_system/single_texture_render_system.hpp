#pragma once

#include "glm/glm.hpp"
#include "simple_render_system.hpp"

struct SingleTextureRenderSystemModifier {
    static constexpr const int poolSize = 20000;
    static void modifyPipeline(PipelineConfigInfo &configInfo) {
        configInfo.inputAssemblyInfo.topology = VkPrimitiveTopology::VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
    }
};

struct SingleTextureRenderSystemPushConstantData {
    glm::mat4 view_proj;
    static VkShaderStageFlags getStageFlags() {
        return VK_SHADER_STAGE_GEOMETRY_BIT;
    }
};

using SingleTextureRenderSystem = SimpleRenderSystem<0, SimplePushConstantInfoList, SingleTextureRenderSystemModifier>;