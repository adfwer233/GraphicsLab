#pragma once

/**
 * @brief New render graph for the graphics lab.
 *
 * @section Functionality Goals
 *
 * 1. Support defining or using a default render graph in the imported project.
 * 2. Support computational passes for applications such as path tracing-related projects.
 * 3. Support "hyper render graph", which refers to the composition of render graphs.
 *
 * @section Goals
 *
 * 1. Separate the render graph from the rendering backend, meaning no Vulkan-related logic appears in the render graph
 * implementation.
 * 2. Support creating the render graph with scripts.
 *
 * @section Design
 *
 * - **RenderPass**: Descriptor of a render pass.
 *     - The `RenderPass` should maintain a set of rendering systems and the record function.
 *     - The `RenderPass` should be customizable and Reflectable, i.e., able to reflect input and output render
 * resources.
 *
 * - **RenderResources**: Descriptor of rendering resources.
 *     - Buffers and textures are sufficient.
 *
 * - **RenderGraph**: Describes the render passes and resources.
 *      - `RenderGraph` should maintain a context (use dynamic reflection) and passes can communicate via this context.
 *
 * - **RenderGraphCompiler**: Computes the `RenderGraph` into a `RenderGraphInstance`.
 *
 * - **RenderGraphInstance**: The executable render graph in the application.
 *     - The `RenderGraphInstance` should maintain a Scene Object, which, in the current framework, is a `SceneTree`.
 *     - Render resource instances are managed by the `RenderGraphInstance`.
 */

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "spdlog/spdlog.h"

#include "render_pass.hpp"
#include "utils/graph.hpp"

#include "language/coroutine/generator.hpp"

namespace GraphicsLab::RenderGraph {

struct RenderGraph {

    RenderGraph() : graph_(10) {
    }

    void add_pass(RenderPass *render_pass, const std::string &pass_name) {
        // add the pass ptr to graph
        render_pass->set_name(pass_name);
        auto idx = graph_.add_node({render_pass});
        pass_name_to_index[pass_name] = idx;
    }

    void add_edge(const std::string &from, const std::string &to) {
        if (pass_name_to_index.contains(from) and pass_name_to_index.contains(to)) {
            graph_.add_directed_edge(pass_name_to_index[from], pass_name_to_index[to], {});
        } else {
            spdlog::warn("Render Graph add_edge failed since {} or {} is not the pass name added to graph", from, to);
            return;
        }
    }

    RenderPass *get_pass(const std::string &pass_name) {
        // return the pass from graph
        if (not pass_name_to_index.contains(pass_name)) {
            spdlog::warn("No pass named {}", pass_name);
            return nullptr;
        }

        return graph_.nodes[pass_name_to_index[pass_name]].data.render_pass;
    }

    size_t get_pass_idx(const std::string &pass_name) {
        if (not pass_name_to_index.contains(pass_name)) {
            spdlog::warn("No pass named {}", pass_name);
            return 0;
        }

        return pass_name_to_index[pass_name];
    }

    Generator<RenderPass *> get_all_passes_generator() {
        for (auto &node : graph_.nodes) {
            co_yield node.data.render_pass;
        }
    }

  private:
    std::string name_;
    std::map<std::string, size_t> pass_name_to_index;

    struct NodeAttachment {
        RenderPass *render_pass;
    };

    struct EdgeAttachment {};

    DirectedGraph<NodeAttachment, EdgeAttachment> graph_;

    friend class RenderGraphCompiler;
};

struct HyperRenderGraph {
    std::vector<RenderGraph *> graphs;

    void render();
};

} // namespace GraphicsLab::RenderGraph
