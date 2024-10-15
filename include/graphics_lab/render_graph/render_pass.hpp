#pragma once

#include <functional>
#include "vkl/core/vkl_render_pass.hpp"

#include "render_context.hpp"

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
    };

    virtual ~RenderPass() = default;

    const std::string_view get_name() {
        return name_;
    }

    virtual void compile(RenderContext* render_context, const CompileData& compile_data) = 0;
    virtual void execute(RenderContext* render_context, const RenderPassExecuteData& execute_data) = 0;

protected:
    RenderPass(VklDevice& device): device_(device) {}

    VklDevice& device_;
    std::string name_;

    std::function<void(void)> record_function_;

    friend class RenderGraph;
};

}
}