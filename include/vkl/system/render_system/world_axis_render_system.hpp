#pragma once

#include "glm/glm.hpp"
#include "simple_render_system.hpp"

struct WorldAxisRenderSystemModifier {
    static constexpr const int poolSize = 20000;
    static void modifyPipeline(PipelineConfigInfo &configInfo) {
        configInfo.inputAssemblyInfo.topology = VkPrimitiveTopology::VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
    }
};

struct WorldAxisRenderSystemPushConstantData {
    glm::mat4 view_proj;
    static VkShaderStageFlags getStageFlags() {
        return VK_SHADER_STAGE_GEOMETRY_BIT;
    }
};

using WorldAxisRenderSystem = SimpleRenderSystem<0, VklPushConstantInfoList<WorldAxisRenderSystemPushConstantData>, WorldAxisRenderSystemModifier>;