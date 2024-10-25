#include "application.hpp"

#include "graphics_lab/render_graph/render_graph.hpp"
#include "graphics_lab/render_graph/render_graph_compiler.hpp"
#include "graphics_lab/render_passes/simple_pass.hpp"
#include "graphics_lab/render_passes/swap_chain_pass.hpp"

Application::~Application() {
}

void Application::run() {
    using namespace GraphicsLab::RenderGraph;

    GLFWwindow *window = window_.getGLFWwindow();

    RenderGraph render_graph;

    SimpleRenderPass simpleRenderPass(device_);
    SwapChainPass swapChainPass(device_, "simple_output");
    render_graph.add_pass(&simpleRenderPass, "simple_pass");
    render_graph.add_pass(&swapChainPass, "swap_chain_pass");

    RenderGraphCompiler compiler(render_graph, device_);
    auto render_graph_instance = compiler.compile(&context);

    while (not glfwWindowShouldClose(window)) {
        glfwPollEvents();

        context.begin_frame();
        render_graph_instance->execute(&context);
        context.end_frame();
    }
}