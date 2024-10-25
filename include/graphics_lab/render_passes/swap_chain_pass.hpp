#pragma once

#include "graphics_lab/render_graph/render_pass.hpp"

namespace GraphicsLab {
namespace RenderGraph {

/**
 * @brief special render pass used to render to swap chain
 *
 * `SpecialChainPass` blit a image to the swap chain rendering context
 */
struct SwapChainPass: public RenderPass {
    virtual void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) override {

    }
};

}
}