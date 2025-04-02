#include "application.hpp"

#include <thread>

#include "vkl/imgui/imgui_context.hpp"

#include "graphics_lab/render_graph/render_graph.hpp"
#include "graphics_lab/render_graph/render_graph_compiler.hpp"
#include "graphics_lab/render_passes/simple_imgui_pass.hpp"
// #include "graphics_lab/render_passes/simple_pass.hpp"
#include "vkl/core/vkl_window.hpp"

#include "scene.hpp"

BezierGeneratorApplication::~BezierGeneratorApplication() {
}

void BezierGeneratorApplication::run() {
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

    GraphicsLab::BezierGenerator::Scene2D scene;
    scene.curves.emplace_back(std::move(std::vector<GraphicsLab::Geometry::BezierCurve2D::PointType>{glm::vec<2, double>{0.0, 0.0}, glm::vec<2, double>{1.0, 1.0}}));

    SceneTree::GeometryNode<GraphicsLab::BezierGenerator::Scene2D> scene_node(std::move(scene));

    BezierRenderPass bezier_render_pass(appContext.device_, &scene_node);

    appContext.renderGraph->add_pass(&bezier_render_pass, "bezier_pass");
    appContext.renderGraph->add_pass(&simple_pass, "simple_pass");
    appContext.renderGraph->add_edge("bezier_pass", "simple_pass");

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