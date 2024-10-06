#pragma once

/**
 * New render graph for the graphics lab.
 *
 * Functionality Goals:
 *
 * 1. Support define or use default render graph in the imported project
 * 2. Support computational pass for applications such as path tracing related projects
 * 3. Support "hyper render graph", that is the composition of render graphs
 *
 * Design Goals:
 *
 * 1. Separate render graph with the rendering backend, i.e. no vulkan-related logic appears in the render graph
 * implementation
 *
 * Design:
 *
 * We got idea from the entity component systems, pass instances and resource instances are stored in a vector
 * `RenderGraph`
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