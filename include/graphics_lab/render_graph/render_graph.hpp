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
 *
 * - **RenderGraphCompiler**: Computes the `RenderGraph` into a `RenderGraphInstance`.
 *
 * - **RenderGraphInstance**: The executable render graph in the application.
 *     - The `RenderGraphInstance` should maintain a Scene Object, which, in the current framework, is a `SceneTree`.
 *     - Render resource instances are managed by the `RenderGraphInstance`.
 */

#include <memory>
#include <string>
#include <vector>

namespace GraphicsLab {

namespace RenderGraph {

struct RenderResource {};
struct RenderResourceInstance {};

struct RenderTexture {};
struct RenderBuffer {};

struct TextureInstance : public RenderResourceInstance {};
struct BufferInstance : public RenderResourceInstance {};

struct Pass {
    enum class Type {
        RenderPass,
        ComputePass
    };

    Type type;
    std::string name;
    size_t index;
    size_t instance_index;

  private:
};

struct ComputePass : Pass {};
struct RenderPass : Pass {};
struct SwapChainPass : Pass {};

struct PassInstance {};

struct ComputePassInstance : public PassInstance {};
struct RenderPassInstance : public PassInstance {};
struct SwapChainPassInstance : public PassInstance {};

struct RenderGraph {

    void create_pass_instances();
    void create_resource_instances();

    void compile();
    void render();

  private:
    std::vector<std::unique_ptr<Pass>> passes;
    std::vector<std::unique_ptr<RenderResource>> resources;

    std::vector<std::unique_ptr<RenderResourceInstance>> resource_instances;
    std::vector<std::unique_ptr<PassInstance>> pass_instances;
};

struct HyperRenderGraph {
    std::vector<RenderGraph *> graphs;

    void render();
};

} // namespace RenderGraph
} // namespace GraphicsLab