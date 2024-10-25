#include "application_experimental.hpp"

#include <thread>

#include "vkl/imgui/imgui_context.hpp"

#include "graphics_lab/render_graph/render_graph.hpp"
#include "graphics_lab/render_graph/render_graph_compiler.hpp"
#include "graphics_lab/render_passes/simple_pass.hpp"
#include "graphics_lab/render_passes/swap_chain_pass.hpp"

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

    SceneTree::VklSceneTree scene_tree(appContext.device_);

    // scene_tree.importFromPath(std::format("{}/nanosuit/nanosuit.obj", DATA_DIR));
    scene_tree.addCameraNode("Camera 1", Camera({0, 0, 50}, {0, 1, 0}));
    scene_tree.addCameraNode("Camera 2", Camera({0, -10, 30}, {0, 1, 0}, {0, -10, 0}));

    UIState state;
    Controller controller(state, scene_tree);
    auto env_injector =
            di::make_injector(di::bind<SceneTree::VklSceneTree>().to(scene_tree), di::bind<VklDevice>().to(appContext.device_),
                              di::bind<UIState>().to(state), di::bind<Controller>().to(controller), di::bind<GraphicsLab::GraphicsLabInternalContext>().to(appContext));

    Controller::controller = &controller;
    glfwSetCursorPosCallback(window, Controller::mouse_callback);
    glfwSetScrollCallback(window, Controller::scroll_callback);
    glfwSetMouseButtonCallback(window, Controller::mouse_button_callback);

    UIManager uiManager(scene_tree, controller, state, appContext);

    InternalSceneRenderPass internalSceneRenderPass(appContext.device_, scene_tree, state);
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

        float currentTime = static_cast<float>(glfwGetTime());
        deltaTime = currentTime - lastFrame;
        lastFrame = currentTime;

        controller.processInput(appContext.window_.getGLFWwindow(), deltaTime);

        ImGui_ImplGlfw_NewFrame();
        ImGui_ImplVulkan_NewFrame();
        ImGui::NewFrame();

        appContext.renderContext.begin_frame();
        appContext.renderGraphInstance->execute(&appContext.renderContext);
        auto render_result = appContext.renderContext.end_frame();

        if (render_result == VK_ERROR_OUT_OF_DATE_KHR || render_result == VK_SUBOPTIMAL_KHR ||
            appContext.window_.wasWindowResized()) {
            appContext.window_.resetWindowResizedFlag();
            appContext.renderContext.recreate_swap_chain();
        }
    }

    vkDeviceWaitIdle(appContext.device_.device());

    ImguiContext::getInstance()->cleanContext();
    SceneTree::VklNodeMeshBuffer<Mesh3D>::free_instance();
    SceneTree::VklNodeMeshBuffer<Wire3D>::free_instance();
}