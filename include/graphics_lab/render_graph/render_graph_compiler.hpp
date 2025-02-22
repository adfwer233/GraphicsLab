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

        // generate the dependency order
        generate_edges();

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
                if (f.get_visibility() == RenderPassReflection::Field::Visibility::Internal) {
                    auto resource = context->resource_manager.add_resource(f);
                }
                if (f.get_visibility() == RenderPassReflection::Field::Visibility::Output) {
                    auto resource = context->resource_manager.add_resource(f);
                }
            }
        }
    }

    void generate_edges() {
        // texture_name -> producer_idx
        std::map<std::string, size_t> producer_map;

        for (auto pass : render_graph_.get_all_passes_generator()) {
            for (auto &f : pass->render_pass_reflect()) {
                if (f.get_visibility() == RenderPassReflection::Field::Visibility::Output) {
                    producer_map[f.get_name()] = render_graph_.get_pass_idx(std::string(pass->get_name()));
                }
            }
        }

        for (auto pass : render_graph_.get_all_passes_generator()) {
            auto consumer_idx = render_graph_.get_pass_idx(std::string(pass->get_name()));
            for (auto &f : pass->render_pass_reflect()) {
                if (f.get_visibility() == RenderPassReflection::Field::Visibility::Input) {
                    if (producer_map.contains(f.get_name())) {
                        render_graph_.graph_.add_directed_edge(producer_map[f.get_name()], consumer_idx, {});
                    }
                }
            }
        }
    }

    void import_resources() {
        /**
         * @todo: support import resources from external files
         */
    }

    void compile_passes(RenderContext *context) {
        // collect all output data
        std::vector<RenderPassReflection::Field> fields;

        for (auto pass : render_graph_.get_all_passes_generator()) {
            for (auto &f : pass->render_pass_reflect()) {
                if (f.get_visibility() == RenderPassReflection::Field::Visibility::Output) {
                    fields.push_back(f);
                }
            }
        }

        for (auto pass : render_graph_.get_all_passes_generator()) {
            RenderPassReflection connected_resources;
            for (auto &f : pass->render_pass_reflect()) {
                if (f.get_visibility() == RenderPassReflection::Field::Visibility::Output) {
                    connected_resources.add_field(f);
                }
                if (f.get_visibility() == RenderPassReflection::Field::Visibility::Input) {
                    for (auto f_iter : fields) {
                        if (f_iter.get_name() == f.get_name()) {
                            RenderPassReflection::Field tmp_field = f_iter;
                            tmp_field.visibility(f.get_visibility());
                            connected_resources.add_field(tmp_field);
                        }
                    }
                }
            }
            pass->compile(context, {connected_resources}); // only add the reflected data
        }

        for (auto pass : render_graph_.get_all_passes_generator())
            pass->post_compile(context);
    }
};

} // namespace RenderGraph
} // namespace GraphicsLab