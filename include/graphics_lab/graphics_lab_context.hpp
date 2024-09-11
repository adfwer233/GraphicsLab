#pragma once

#include "vkl/render_graph/render_graph.hpp"

struct GraphicsLabContext {
    VklDevice* device;
    RenderGraphPassDerived<RenderGraphRenderPass>* renderGraphRenderPass;
};
