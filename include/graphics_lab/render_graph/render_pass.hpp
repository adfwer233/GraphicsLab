#pragma once

#include <functional>
#include <memory>

#include "vkl/core/vkl_render_pass.hpp"
#include "vkl/core/vkl_framebuffer.hpp"

#include "render_context.hpp"
#include "render_pass_reflection.hpp"

namespace GraphicsLab {
namespace RenderGraph {

/**
 * @brief: Data passed to the render pass when execute.
 */
struct RenderPassExecuteData {
    /**
     * pointers to render resources.
     */
};

/**
 * @brief Render Graphics Descriptor in Graphics Lab Render Graph
 */
struct RenderPass {

    struct CompileData {
        /**
         * data used to compile render pass.
         */

        RenderPassReflection connected_resources;
    };

    virtual ~RenderPass() = default;

    const std::string_view get_name() {
        return name_;
    }

    virtual void compile(RenderContext *render_context, const CompileData &compile_data) {
        // create VkRenderPass and VkFrameBuffer

    }

    virtual void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) = 0;

    virtual RenderPassReflection render_pass_reflect() = 0;
  protected:
    RenderPass(VklDevice &device) : device_(device) {
    }

    VklDevice &device_;
    std::string name_;

    std::function<void(void)> record_function_;

    std::unique_ptr<VklRenderPass> vkl_render_pass;
    std::unique_ptr<VklFramebuffer> vkl_frame_buffer;

    friend class RenderGraph;
};

} // namespace RenderGraph
} // namespace GraphicsLab