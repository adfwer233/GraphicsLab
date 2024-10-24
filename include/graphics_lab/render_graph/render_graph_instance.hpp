#pragma once

#include <string>

#include "render_pass.hpp"

namespace GraphicsLab {
namespace RenderGraph {

/**
 * @brief Render graph executable instance used in main rendering loop
 * Render graph instance is generated by the `RenderGraphCompiler`
 */
struct RenderGraphInstance {
    struct Pass {
        std::string name;
        RenderPass* pass;
    };

    void execute(RenderContext *context) {
        for (auto pass: executionSequence_) {
            RenderPassExecuteData render_data;
            pass.pass->execute(context, render_data);
        }
    }
private:
    std::vector<Pass> executionSequence_;

    friend class RenderGraphCompiler;
};

}
}
