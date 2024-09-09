#include "application.hpp"

#include "vkl/render_graph/render_graph.hpp"
#include "vkl/scene/vkl_scene.hpp"
#include "vkl/system/render_system/simple_render_system.hpp"

#include "vkl/imgui/imgui_context.hpp"

#include "geometry/geometry.hpp"
#include "vkl/scene_tree/vkl_geometry_mesh.hpp"
#include "vkl/scene_tree/vkl_mesh.hpp"
#include "vkl/scene_tree/vkl_scene_tree.hpp"

#include "controller/controller.hpp"
#include "render/render_manager.hpp"
#include "ui/ui_manager.hpp"

#include "boost/di.hpp"

namespace di = boost::di;

Application::~Application() = default;

void Application::run() {
    GLFWwindow *window = window_.getGLFWwindow();

    SceneTree::VklSceneTree scene_tree(device_);

    scene_tree.importFromPath(std::format("{}/nanosuit/nanosuit.obj", DATA_DIR));
    scene_tree.addCameraNode("Camera 1", Camera({0, 0, 50}, {0, 1, 0}));
    scene_tree.addCameraNode("Camera 2", Camera({0, -10, 30}, {0, 1, 0}, {0, -10, 0}));

    UIState state;
    Controller controller(state, scene_tree);
    auto env_injector =
        di::make_injector(di::bind<SceneTree::VklSceneTree>().to(scene_tree), di::bind<VklDevice>().to(device_),
                          di::bind<UIState>().to(state),
                          di::bind<Controller>().to(controller));

    spdlog::info("[controller] {}", (void *)&controller);

    // glfwSetWindowUserPointer(window, &controller);
    Controller::controller = &controller;
    glfwSetCursorPosCallback(window, Controller::mouse_callback);
    glfwSetScrollCallback(window, Controller::scroll_callback);
    glfwSetMouseButtonCallback(window, Controller::mouse_button_callback);

    RenderGraphDescriptor graphDescriptor;
    auto renderPassManager = env_injector.create<RenderPassManager>();

    renderPassManager.descriptorStage(graphDescriptor);

    RenderGraph renderGraph(device_, {WIDTH, HEIGHT}, &graphDescriptor);
    renderGraph.createLayouts();
    renderGraph.createInstances();

    auto imguiContext = std::make_unique<ImguiContext>(device_, window, renderGraph.swapChain_->getRenderPass());

    renderPassManager.instanceStage(renderGraph);

    float deltaTime = 0, lastFrame = 0;

    while (not glfwWindowShouldClose(window)) {
        glfwPollEvents();

        float currentTime = static_cast<float>(glfwGetTime());
        deltaTime = currentTime - lastFrame;
        lastFrame = currentTime;

        // controller.processInput(window_.getGLFWwindow(), deltaTime);

        uint32_t currentFrame = renderGraph.beginFrame();

        ImGui_ImplGlfw_NewFrame();
        ImGui_ImplVulkan_NewFrame();
        ImGui::NewFrame();

        auto render_result = renderGraph.render(renderGraph.commandBuffers[currentFrame], currentFrame);

        if (render_result == VK_ERROR_OUT_OF_DATE_KHR || render_result == VK_SUBOPTIMAL_KHR ||
            window_.wasWindowResized()) {
            window_.resetWindowResizedFlag();
            auto extent = window_.getExtent();
            while (extent.width == 0 || extent.height == 0) {
                extent = window_.getExtent();
                glfwWaitEvents();
            }
            renderGraph.recreateSwapChain(extent);
            glfwSetWindowUserPointer(window, &controller);
            vkDeviceWaitIdle(device_.device());
        }

        renderGraph.endFrame();
    }

    vkDeviceWaitIdle(device_.device());

    imguiContext.reset();
    SceneTree::VklNodeMeshBuffer<Mesh3D>::free_instance();
}