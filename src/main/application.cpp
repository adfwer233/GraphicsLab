#include "application.hpp"

#include <thread>

#include "vkl/imgui/imgui_context.hpp"

#include "geometry/constructor/directional_field_constructor.hpp"

#include "graphics_lab/render_graph/render_graph.hpp"
#include "graphics_lab/render_graph/render_graph_compiler.hpp"
#include "graphics_lab/render_passes/simple_pass.hpp"

#include "vkl/core/vkl_window.hpp"

#include "render/internal_render_pass/3d_scene_internal_pass.hpp"
#include "render/internal_render_pass/internal_imgui_pass.hpp"

ApplicationExperimental::~ApplicationExperimental() {
}

void ApplicationExperimental::run() {
    using namespace GraphicsLab::RenderGraph;
    using namespace std::chrono_literals;

    // customize the log manager
    auto &logManager = LogManager::getInstance();

    GLFWwindow *window = appContext.window_.getGLFWwindow();

    appContext.sceneTree = std::make_unique<SceneTree::VklSceneTree>(appContext.device_);

    appContext.sceneTree->root->name = "test";

    if (appOption.load_obj_path.has_value()) {
        appContext.sceneTree->importFromPath(appOption.load_obj_path.value());
    }

    appContext.sceneTree->addCameraNode("Camera 1", Camera({0, 0, 50}, {0, 1, 0}));
    appContext.sceneTree->addCameraNode("Camera 2", Camera({0, -10, 30}, {0, 1, 0}, {0, -10, 0}));

    auto directional_field = GraphicsLab::Geometry::SimpleDirectionalFieldConstructor::create();

    appContext.sceneTree->addGeometryNode(std::move(directional_field), "dir");

    UIState state;
    Controller controller(state, *appContext.sceneTree);
    ControllerCallbackHandler::internal_controller = &controller;

    glfwSetCursorPosCallback(window, ControllerCallbackHandler::mouse_callback);
    glfwSetScrollCallback(window, ControllerCallbackHandler::scroll_callback);
    glfwSetMouseButtonCallback(window, ControllerCallbackHandler::mouse_button_callback);

    UIManager uiManager(controller, state, appContext);

    InternalSceneRenderPass internalSceneRenderPass(appContext.device_, *appContext.sceneTree, state,
                                                    uiManager.renderResources);
    InternalImguiPass internalImguiPass(appContext.device_, uiManager);

    internalSceneRenderPass.set_extent(2048, 2048);
    internalImguiPass.set_extent(WIDTH, HEIGHT);

    appContext.renderGraph->add_pass(&internalSceneRenderPass, "internal_scene_render_pass");
    appContext.renderGraph->add_pass(&internalImguiPass, "internal_imgui_pass");
    appContext.renderGraph->add_edge("internal_scene_render_pass", "internal_imgui_pass");

    appContext.compileRenderGraph();

    float deltaTime = 0, lastFrame = 0;

#ifdef RENDERDOC_DIR
    GraphicsLab::RenderDocApi::initialize_render_doc();
#endif

    while (not glfwWindowShouldClose(window)) {
        if (appContext.newRenderGraphInstance != nullptr) {
            appContext.renderGraphInstance = std::move(appContext.newRenderGraphInstance);
        }

        glfwPollEvents();

        auto currentTime = static_cast<float>(glfwGetTime());
        deltaTime = currentTime - lastFrame;
        lastFrame = currentTime;

        glfwSetWindowTitle(window, std::format("Graphics Lab {:.2f} FPS", 1 / deltaTime).c_str());
        controller.process_keyboard_input(appContext.window_.getGLFWwindow(), deltaTime);

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
    SceneTree::VklNodeMeshBuffer<Mesh3D>::free_instance();
    SceneTree::VklNodeMeshBuffer<Wire3D>::free_instance();
}