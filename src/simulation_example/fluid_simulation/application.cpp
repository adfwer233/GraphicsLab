#include "application.hpp"

#include <thread>

#include "vkl/imgui/imgui_context.hpp"

#include "graphics_lab/render_graph/render_graph.hpp"
#include "graphics_lab/render_graph/render_graph_compiler.hpp"
#include "graphics_lab/render_passes/simple_imgui_pass.hpp"
#include "graphics_lab/render_passes/simple_pass.hpp"
#include "vkl/core/vkl_window.hpp"

#include "graphics_lab/render_passes/grid2d_render_pass.hpp"

#include "simulation/fluid/base_fluid_simulator.hpp"

#include "vkl/utils/color_interpolator.hpp"

enum class VisualizeQuantity {
    Density,
    Curl
};

FluidSimulationApplication::~FluidSimulationApplication() {
}

void FluidSimulationApplication::run() {
    using namespace GraphicsLab::RenderGraph;
    using namespace std::chrono_literals;

    // customize the log manager
    auto &logManager = LogManager::getInstance();

    GLFWwindow *window = appContext.window_.getGLFWwindow();

    ImguiContext::set_font = false;
    ImguiContext::getInstance(appContext.device_, appContext.window_.getGLFWwindow(),
                              appContext.renderContext.get_swap_chain_render_pass());

    GraphicsLab::Simulation::Grid2D<float> grid(1024, 1024);

    for (int i = 0; i < 1024; i++) {
        for (int j = 0; j < 1024; j++) {
            if (std::pow(i - 512, 2) + std::pow(j - 512, 2) <= std::pow(100, 2)) {
                grid(i, j) = 0.5f;
            }
        }
    }

    ViridisInterpolator viridis_interpolator(0.0, 1.0);
    ThreePointInterpolator three_point_interpolator({1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, -0.5,
                                                    0.5);

    GraphicsLab::Simulation::BaseFluidSimulator simulator(512, 512);

    for (int i = 0; i < 512; i++) {
        for (int j = 0; j < 512; j++) {
            if (std::pow(i - 100, 2) + std::pow(j - 256, 2) <= std::pow(50, 2)) {
                simulator.staggered_grid.density(i, j) = 1.0f;
                simulator.staggered_grid.velocity_x(i, j) = 5.0f;
            }
        }
    }

    // for (int i = 0; i < 512; i++) {
    //     for (int j = 0; j < 512; j++) {
    //         if (std::pow(i - 300, 2) + std::pow(j - 256, 2) <= std::pow(10, 2)) {
    //             simulator.density(i, j) = 1.0f;
    //             simulator.velocity_x(i, j) = -0.0f;
    //         }
    //     }
    // }

    VisualizeQuantity visualization_quantity = VisualizeQuantity::Density;
    const char *items[] = {"Density", "Curl"};

    SimpleImGuiPass simple_pass(appContext.device_, [&]() {
        ImGui::Begin("Settings");
        ImGui::Combo("Select Quantity to Visualize", reinterpret_cast<int *>(&visualization_quantity), items,
                     IM_ARRAYSIZE(items));
        ImGui::End();
    });

    Grid2DRenderPass grid_2d_render_pass(
        appContext.device_,
        [&]() -> GraphicsLab::Simulation::Grid2D<float> & {
            if (visualization_quantity == VisualizeQuantity::Density) {
                return simulator.staggered_grid.density;
            } else if (visualization_quantity == VisualizeQuantity::Curl) {
                return simulator.curl;
            }
        },
        [&](float x) {
            if (visualization_quantity == VisualizeQuantity::Density) {
                return glm::vec4(viridis_interpolator.interpolate(x), 1.0f);
            } else if (visualization_quantity == VisualizeQuantity::Curl) {
                return glm::vec4(three_point_interpolator.interpolate(x), 1.0f);
            }
        });

    simple_pass.set_extent(WIDTH, HEIGHT);

    appContext.renderGraph->add_pass(&grid_2d_render_pass, "grid_pass");
    appContext.renderGraph->add_pass(&simple_pass, "simple_pass");
    appContext.renderGraph->add_edge("grid_pass", "simple_pass");
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

        simulator.step();

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