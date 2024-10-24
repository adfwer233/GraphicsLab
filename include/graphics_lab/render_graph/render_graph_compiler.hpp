#pragma once

#include <memory>

#include "render_context.hpp"
#include "render_graph.hpp"
#include "render_graph_instance.hpp"

namespace GraphicsLab {
namespace RenderGraph {

struct RenderGraphCompiler {
    std::unique_ptr<RenderGraphInstance> compile(RenderContext *context) {
        auto render_graph_instance = std::make_unique<RenderGraphInstance>();

        // create rendering resources
        create_resources(context);

        // loading resources
        import_resources();

        // compile render passes
        compile_passes(context);

        // generate execution sequence
        auto execution_order_index = render_graph_.graph_.topological_sort();

        for (auto idx : execution_order_index) {
            auto pass = render_graph_.graph_.nodes[idx].data.render_pass;
            render_graph_instance->executionSequence_.push_back({pass->name_, pass});
        }

        return std::move(render_graph_instance);
    }

    explicit RenderGraphCompiler(RenderGraph &render_graph, VklDevice &device)
        : render_graph_(render_graph), device_(device) {
    }

  private:
    VklDevice &device_;
    RenderGraph &render_graph_;

    /**
     * create resources with the information provided in render graph description.
     */
    void create_resources(RenderContext *context) {
        for (auto pass : render_graph_.get_all_passes_generator()) {
            for (auto &f : pass->render_pass_reflect()) {
                auto resource = context->resource_manager.add_resource(f);
            }
        }
    }

    void import_resources() {
        /**
         * @todo: support import resources from external files
         */
    }

    void compile_passes(RenderContext *context) {
        for (auto pass : render_graph_.get_all_passes_generator()) {
            pass->compile(context, {pass->render_pass_reflect()}); // only add the reflected data
        }
    }
};

} // namespace RenderGraph
} // namespace GraphicsLab