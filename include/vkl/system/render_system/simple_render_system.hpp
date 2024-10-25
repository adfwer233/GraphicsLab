#pragma once

#include "vkl/core/vkl_frame_info.hpp"
#include "vkl/core/vkl_graphics_pipeline.hpp"

#include <format>
#include <iostream>
#include <memory>
#include <optional>

#include "render_system_base.hpp"
#include "vkl/core/vkl_push_constant.hpp"
#include "vkl/templates/vkl_concept.hpp"

#ifndef SHADER_DIR
#define SHADER_DIR "./shader/"
#endif

template <typename T>
concept VklPipelineModifierType =
    requires(PipelineConfigInfo &pipelineConfigInfo) { T::modifyPipeline(pipelineConfigInfo); };

struct NullPipelineModifier {
    static void modifyPipeline(PipelineConfigInfo &configInfo) {
    }
};

template <uint32_t Subpass = 0, VklPushConstantInfoListConcept PushConstantInfoList = SimplePushConstantInfoList,
          VklPipelineModifierType PipelineModifierType = NullPipelineModifier>
class SimpleRenderSystem : public BaseRenderSystem {
  private:
    std::string vertex_shader_path_, fragment_shader_path_, geometry_shader_path_;

    VklDevice &device_;

    std::vector<VkPushConstantRange> pushConstantRanges_;

    void createPipelineLayout();
    void createPipeline(VkRenderPass renderPass, std::vector<VklShaderModuleInfo> shaderInfos);

  public:
    std::unique_ptr<VklGraphicsPipeline> pipeline_;
    VkPipelineLayout pipelineLayout_{};
    SimpleRenderSystem(VklDevice &device, VkRenderPass renderPass, std::vector<VklShaderModuleInfo> shaderInfos);

    SimpleRenderSystem(const SimpleRenderSystem &) = delete;
    SimpleRenderSystem operator=(const SimpleRenderSystem &) = delete;

    ~SimpleRenderSystem();

    template <VklRenderable ModelType> void renderObject(FrameInfo<ModelType> &frameInfo);
    template <VklRenderable ModelType>
    void renderObject(FrameInfo<ModelType> &frameInfo, PushConstantInfoList pushData);

    void bindPipeline(VkCommandBuffer commandBuffer) {
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_->graphicsPipeline_);
    }
};

#include "simple_render_system.hpp.impl"