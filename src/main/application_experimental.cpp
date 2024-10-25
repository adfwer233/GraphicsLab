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

    GLFWwindow *window = window_.getGLFWwindow();

    SceneTree::VklSceneTree scene_tree(device_);

    // scene_tree.importFromPath(std::format("{}/nanosuit/nanosuit.obj", DATA_DIR));
    scene_tree.addCameraNode("Camera 1", Camera({0, 0, 50}, {0, 1, 0}));
    scene_tree.addCameraNode("Camera 2", Camera({0, -10, 30}, {0, 1, 0}, {0, -10, 0}));

    UIState state;
    Controller controller(state, scene_tree);
    auto env_injector =
            di::make_injector(di::bind<SceneTree::VklSceneTree>().to(scene_tree), di::bind<VklDevice>().to(device_),
                              di::bind<UIState>().to(state), di::bind<Controller>().to(controller));

    Controller::controller = &controller;
    glfwSetCursorPosCallback(window, Controller::mouse_callback);
    glfwSetScrollCallback(window, Controller::scroll_callback);
    glfwSetMouseButtonCallback(window, Controller::mouse_button_callback);

    UIManager uiManager(scene_tree, controller, state);

    GraphicsLab::RenderGraph::RenderGraph render_graph;

    InternalSceneRenderPass internalSceneRenderPass(device_, scene_tree, state);
    InternalImguiPass internalImguiPass(device_, uiManager);

    internalSceneRenderPass.set_extent(2048, 2048);
    internalImguiPass.set_extent(WIDTH, HEIGHT);

    render_graph.add_pass(&internalSceneRenderPass, "internal_scene_render_pass");
    render_graph.add_pass(&internalImguiPass, "internal_imgui_pass");
    render_graph.add_edge("internal_scene_render_pass", "internal_imgui_pass");

    RenderGraphCompiler compiler(render_graph, device_);
    auto render_graph_instance = compiler.compile(&context);

    float deltaTime = 0, lastFrame = 0;

#ifdef RENDERDOC_DIR
    GraphicsLab::RenderDocApi::initialize_render_doc();
#endif

    while (not glfwWindowShouldClose(window)) {
        glfwPollEvents();

        float currentTime = static_cast<float>(glfwGetTime());
        deltaTime = currentTime - lastFrame;
        lastFrame = currentTime;

        controller.processInput(window_.getGLFWwindow(), deltaTime);

        ImGui_ImplGlfw_NewFrame();
        ImGui_ImplVulkan_NewFrame();
        ImGui::NewFrame();

        context.begin_frame();
        render_graph_instance->execute(&context);
        auto render_result = context.end_frame();

        if (render_result == VK_ERROR_OUT_OF_DATE_KHR || render_result == VK_SUBOPTIMAL_KHR ||
            window_.wasWindowResized()) {
            window_.resetWindowResizedFlag();
            context.recreate_swap_chain();
        }
    }

    vkDeviceWaitIdle(device_.device());
    SceneTree::VklNodeMeshBuffer<Mesh3D>::free_instance();
    SceneTree::VklNodeMeshBuffer<Wire3D>::free_instance();
}