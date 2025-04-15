#include "application.hpp"

#include <thread>

#include "vkl/imgui/imgui_context.hpp"

#include "graphics_lab/render_graph/render_graph.hpp"
#include "graphics_lab/render_graph/render_graph_compiler.hpp"
#include "graphics_lab/render_passes/simple_imgui_pass.hpp"
// #include "graphics_lab/render_passes/simple_pass.hpp"
#include "vkl/core/vkl_window.hpp"

#include "random_curve_generator.hpp"

#include "scene.hpp"
#include "ui_state.hpp"

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

    UIState ui_state{.ubo = {
                         .zoom = 0.5f,
                         .offset_x = 0.0f,
                         .offset_y = 0.0f,
                     }};

    GraphicsLab::BezierGenerator::Scene2D scene;

    scene.curves = GraphicsLab::RandomCurveGenerator::generate_biperiodic_curves();

    GraphicsLab::BezierGenerator::Data data;
    for (const auto &curve : scene.curves) {
        data.paths.push_back({{curve}});
    }

    auto data_json = StaticReflect::serialization(data);
    spdlog::info(data_json.dump(4));

    spdlog::info("Generate Dataset A");

    SceneTree::GeometryNode<GraphicsLab::BezierGenerator::Scene2D> scene_node(std::move(scene));

    BezierRenderPass bezier_render_pass(appContext.device_, &scene_node, &ui_state);

    SimpleImGuiPass simple_pass(appContext.device_, [&]() {
        ImGui::Begin("Dataset");
        {
            if (ImGui::Button("Create Dataset A")) {
                for (int i = 0; i <= 25; i++) {
                    spdlog::info("Generate Dataset A: {}", i);
                    auto curves = GraphicsLab::RandomCurveGenerator::generate_uniperiodic_curves();
                    GraphicsLab::BezierGenerator::Data param_data;
                    for (const auto &curve : curves) {
                        param_data.paths.push_back({{curve}});
                    }
                    auto data_json = StaticReflect::serialization(param_data);
                    auto data_json_str = data_json.dump(4);

                    std::ofstream file(std::format("{}.json", i));
                    file << data_json_str;
                    file.close();
                }
            }
        }
        ImGui::End();
    });
    simple_pass.set_extent(WIDTH, HEIGHT);

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