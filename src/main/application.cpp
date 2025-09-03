#include "graphics_lab/application.hpp"

#include <cpptrace/cpptrace.hpp>
#include <thread>

#include "vkl/imgui/imgui_context.hpp"
#include "vkl/core/vkl_window.hpp"

#include "geometry/constructor/directional_field_constructor.hpp"

#include "graphics_lab/render_graph/render_graph.hpp"
#include "graphics_lab/render_graph/render_graph_compiler.hpp"
#include "graphics_lab/render_passes/simple_pass.hpp"
#include "lua/lua_binding.hpp"
#include "lua/scene_binding.hpp"

#include "render/internal_render_pass/2d_scene_internal_pass.hpp"
#include "render/internal_render_pass/3d_scene_internal_pass.hpp"
#include "render/internal_render_pass/internal_imgui_pass.hpp"

#include <cpptrace/from_current.hpp>

GraphicsLabApplication::~GraphicsLabApplication() {
}

void GraphicsLabApplication::initialize() {
    using namespace GraphicsLab::RenderGraph;
    using namespace std::chrono_literals;

    // hook std::terminate
    std::set_terminate([] {
        spdlog::critical("Unhandled exception! Dumping stacktrace:");
        cpptrace::from_current_exception().print();
        std::abort();
    });

    // customize the log manager
    auto &logManager = LogManager::getInstance();

    appContext.sceneTree = std::make_unique<SceneTree::VklSceneTree>(appContext.device_);
    appContext.sceneTree->root->name = "test";

    // initialize lua context
    appContext.lua.open_libraries(sol::lib::base);
    GraphicsLab::LuaBinding::bind(appContext.lua);
    GraphicsLab::LuaSceneInterface::bind(appContext.lua);

    GraphicsLab::LuaSceneInterface sceneInterface(appContext.sceneTree.get());

    // Pass it into Lua as a global
    appContext.lua["scene"] = appContext.sceneTree.get();
    appContext.lua["sceneInterface"] = &sceneInterface;

    appContext.lua.script(R"(
        local points = {
            vec2.new(0.0, 0.0),
            vec2.new(0.0, 0.1),
            vec2.new(0.0, 0.2)
        }

        local v3 = vec3.new(1,2,3)
        print("vec3:", v3.x, v3.y, v3.z)

        local v4 = vec4.new(1,2,3,4)
        print("vec4:", v4.x, v4.y, v4.z, v4.w)

        sceneInterface:add_point_cloud_2d(points)
    )");

    if (appOption.load_obj_path.has_value()) {
        appContext.sceneTree->importFromPath(appOption.load_obj_path.value());
    }

    appContext.sceneTree->addCameraNode("Camera 1", Camera({0, 0, 50}, {0, 1, 0}));
    appContext.sceneTree->addCameraNode("Camera 2", Camera({0, -10, 30}, {0, 1, 0}, {0, -10, 0}));
}

void GraphicsLabApplication::run() {
    using namespace GraphicsLab::RenderGraph;
    using namespace std::chrono_literals;

    GLFWwindow *window = appContext.window_.getGLFWwindow();

    glfwSetCursorPosCallback(window, ControllerCallbackHandler::mouse_callback);
    glfwSetScrollCallback(window, ControllerCallbackHandler::scroll_callback);
    glfwSetMouseButtonCallback(window, ControllerCallbackHandler::mouse_button_callback);

    UIState &state = AutoSerializeSingleton<UIState, "UIState">::instance();
    Controller controller(state, *appContext.sceneTree);
    ControllerCallbackHandler::internal_controller = &controller;
    UIManager uiManager(controller, state, appContext);

    InternalSceneRenderPass internalSceneRenderPass(appContext.device_, *appContext.sceneTree, state,
                                                    uiManager.renderResources);
    InternalImguiPass internalImguiPass(appContext.device_, uiManager);
    InternalScene2DRenderPass internalScene2DRenderPass(appContext.device_, *appContext.sceneTree, state,
                                                        uiManager.renderResources);

    internalSceneRenderPass.set_extent(2048, 2048);
    internalImguiPass.set_extent(WIDTH, HEIGHT);

    appContext.renderGraph->add_pass(&internalScene2DRenderPass, "internal_scene_2d_render_pass");
    appContext.renderGraph->add_pass(&internalSceneRenderPass, "internal_scene_render_pass");
    appContext.renderGraph->add_pass(&internalImguiPass, "internal_imgui_pass");

    appContext.renderGraph->add_edge("internal_scene_2d_render_pass", "internal_imgui_pass");
    appContext.renderGraph->add_edge("internal_scene_render_pass", "internal_imgui_pass");

    appContext.compileRenderGraph();

    if (projectFactory != nullptr) {
        state.project = projectFactory();
        state.project->updateContext(GraphicsLabContext(&appContext.sceneTree->device_, appContext.sceneTree.get(),
                                                        &LogManager::getInstance(), &state, &appContext));
        state.project->afterLoad();
        state.project_load_by_factory = true;
        ControllerCallbackHandler::project_controller = state.project->getController();
    }

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