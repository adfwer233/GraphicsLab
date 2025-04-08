#include "application.hpp"

#include <thread>

#include "vkl/imgui/imgui_context.hpp"

#include "graphics_lab/render_graph/render_graph.hpp"
#include "graphics_lab/render_graph/render_graph_compiler.hpp"
#include "graphics_lab/render_passes/simple_imgui_pass.hpp"
// #include "graphics_lab/render_passes/simple_pass.hpp"
#include "vkl/core/vkl_window.hpp"

#include "geometry/parametric_intersector/torus_torus_intersector.hpp"

#include "render_pass/parametric_space_render_pass.hpp"

IntersectorDemoApplication::~IntersectorDemoApplication() {
}

void IntersectorDemoApplication::run() {
    using namespace GraphicsLab::RenderGraph;
    using namespace std::chrono_literals;

    // customize the log manager
    auto &logManager = LogManager::getInstance();

    GLFWwindow *window = appContext.window_.getGLFWwindow();

    ImguiContext::set_font = false;
    ImguiContext::getInstance(appContext.device_, appContext.window_.getGLFWwindow(),
                              appContext.renderContext.get_swap_chain_render_pass());

    SimpleImGuiPass simple_pass(appContext.device_);

    simple_pass.set_extent(WIDTH, HEIGHT);

    GraphicsLab::Geometry::Torus torus1({0.0f, 0.0f, 0.0f}, 1.0f, 0.5f, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f});
    GraphicsLab::Geometry::Torus torus2({2.5f, 0.0f, 0.0f}, 1.0f, 0.5f, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f});

    auto res = GraphicsLab::Geometry::TorusTorusIntersector::intersect(torus1, torus2);
    for (auto &item : res) {
        spdlog::info("intersect param1: {}, {}", item.param1.x, item.param2.y);
    }

    spdlog::info("intersect num: {}", res.size());

    ParametricSpaceScene scene;

    for (auto item : res) {
        scene.points.push_back(item.param1);
    }

    spdlog::info("ts {}", scene.points.size());

    SceneTree::GeometryNode<ParametricSpaceScene> scene_node(std::move(scene));
    ParametricSpaceRenderPass parametric_space_render_pass(appContext.device_, &scene_node);

    appContext.renderGraph->add_pass(&parametric_space_render_pass, "parametric_space_render_pass");
    appContext.renderGraph->add_pass(&simple_pass, "simple_pass");
    appContext.renderGraph->add_edge("parametric_space_render_pass", "simple_pass");

    spdlog::info("here");
    appContext.compileRenderGraph();
    float deltaTime = 0, lastFrame = 0;

    while (not glfwWindowShouldClose(window)) {
        if (appContext.newRenderGraphInstance != nullptr) {
            appContext.renderGraphInstance = std::move(appContext.newRenderGraphInstance);
        }

        glfwPollEvents();

        auto currentTime = static_cast<float>(glfwGetTime());
        deltaTime = currentTime - lastFrame;
        lastFrame = currentTime;

        glfwSetWindowTitle(window, std::format("Graphics Lab {:.2f} FPS", 1 / deltaTime).c_str());

        ImGui_ImplGlfw_NewFrame();
        ImGui_ImplVulkan_NewFrame();
        ImGui::NewFrame();

        appContext.renderContext.begin_frame();
        {
            std::scoped_lock instanceLock(appContext.renderGraphInstanceMutex);
            appContext.renderGraphInstance->execute(&appContext.renderContext);
        }

        auto render_result = appContext.renderContext.end_frame();

        if (render_result == VK_ERROR_OUT_OF_DATE_KHR || render_result == VK_SUBOPTIMAL_KHR ||
            appContext.window_.wasWindowResized()) {
            appContext.window_.resetWindowResizedFlag();
            appContext.renderContext.recreate_swap_chain();
            vkDeviceWaitIdle(appContext.device_.device());
        }
    }

    vkDeviceWaitIdle(appContext.device_.device());

    ImguiContext::getInstance()->cleanContext();
}