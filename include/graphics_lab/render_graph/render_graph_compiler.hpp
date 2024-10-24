#pragma once

#include <memory>

#include "render_context.hpp"
#include "render_graph_instance.hpp"

namespace GraphicsLab {
namespace RenderGraph {

struct RenderGraph;

struct RenderGraphCompiler {
    std::unique_ptr<RenderGraphInstance> compile(RenderContext* context, RenderGraph& renderGraph);

private:
    RenderGraph& render_graph_;

    void compile_passes(RenderContext* context) {

    }
};

}
}