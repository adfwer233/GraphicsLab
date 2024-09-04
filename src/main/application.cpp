#include "application.hpp"

#include "vkl/render_graph/render_graph.hpp"
#include "vkl/scene/vkl_scene.hpp"
#include "vkl/system/render_system/simple_render_system.hpp"

#include "vkl/imgui/imgui_context.hpp"

#include "geometry/geometry.hpp"
#include "vkl/scene_tree/vkl_geometry_mesh.hpp"
#include "vkl/scene_tree/vkl_mesh.hpp"
#include "vkl/scene_tree/vkl_scene_tree.hpp"

#include "ui/ui_manager.hpp"

Application::~Application() {
}

void Application::run() {
    GLFWwindow *window = window_.getGLFWwindow();

    VklScene scene(device_, {0, 0, 20}, {0, 1, 0});

    VklModel::BuilderFromImmediateData builder;
    builder.vertices = {
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0}},
        {{0.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0}},
        {{1.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0}},
        {{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0}},
    };

    SceneTree::VklSceneTree scene_tree(device_);
    scene_tree.importFromPath(std::format("{}/nanosuit/nanosuit.obj", DATA_DIR));

    scene.addObject(builder);

    RenderGraphDescriptor graphDescriptor;

    auto output_texture = graphDescriptor.attachment<RenderGraphTextureAttachment>("output_image");
    output_texture->isSwapChain = true;
    output_texture->format = VK_FORMAT_R8G8B8A8_SRGB;
    output_texture->width = 1024;
    output_texture->height = 1024;

    auto render_texture = graphDescriptor.attachment<RenderGraphTextureAttachment>("render_result");
    render_texture->isSwapChain = false;
    render_texture->input_flag = true;
    render_texture->format = VK_FORMAT_R8G8B8A8_SRGB;
    render_texture->width = 1024;
    render_texture->height = 1024;

    auto simple_render_pass = graphDescriptor.pass<RenderGraphRenderPass>("simple_render_pass");
    simple_render_pass->outTextureAttachmentDescriptors.push_back(render_texture);
    simple_render_pass->width = 1024;
    simple_render_pass->height = 1024;

    auto imgui_render_pass = graphDescriptor.pass<RenderGraphRenderPass>("imgui_render_pass");
    imgui_render_pass->outTextureAttachmentDescriptors.push_back(output_texture);
    imgui_render_pass->inTextureAttachmentDescriptors.push_back(render_texture);
    imgui_render_pass->width = 1024;
    imgui_render_pass->height = 1024;
    imgui_render_pass->is_submit_pass = true;

    RenderGraph renderGraph(device_, {WIDTH, HEIGHT}, &graphDescriptor);

    renderGraph.createLayouts();

    renderGraph.createInstances();

    auto simple_render_pass_obj = renderGraph.getPass<RenderGraphRenderPass>("simple_render_pass");
    auto imgui_render_pass_obj = renderGraph.getPass<RenderGraphRenderPass>("imgui_render_pass");

    imgui_render_pass_obj->is_submit_pass = true;

    auto simple_render_system = simple_render_pass_obj->getRenderSystem<SimpleRenderSystem<>>(
        device_, "simple_render_system",
        {{std::format("{}/simple_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
         {std::format("{}/simple_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

    simple_render_pass_obj->recordFunction = [&](VkCommandBuffer commandBuffer, uint32_t frame_index) {
        GlobalUbo ubo{};

        ubo.view = scene.camera.get_view_transformation();
        ubo.proj = scene.camera.get_proj_transformation();
        ubo.model = glm::mat4(1.0f);
        ubo.pointLight = scene.pointLight;
        ubo.cameraPos = scene.camera.position;

        auto key = simple_render_system->descriptorSetLayout->descriptorSetLayoutKey;

        auto mesh3d_buffer = SceneTree::VklNodeMeshBuffer<Mesh3D>::instance();
        for (auto mesh3d_nodes: scene_tree.traverse_geometry_nodes<Mesh3D>()) {
            auto node_mesh = mesh3d_buffer->getGeometryModel(device_, mesh3d_nodes);

            if (node_mesh->mesh->uniformBuffers.contains(key)) {
                node_mesh->mesh->uniformBuffers[key][frame_index]->writeToBuffer(&ubo);
                node_mesh->mesh->uniformBuffers[key][frame_index]->flush();
            }

            FrameInfo<std::decay_t<decltype(*node_mesh)>::render_type> frameInfo{
                    .frameIndex = static_cast<int>(frame_index) % 2,
                    .frameTime = 0,
                    .commandBuffer = commandBuffer,
                    .camera = scene.camera,
                    .model = *node_mesh->mesh.get(),
            };

            simple_render_system->renderObject(frameInfo);
        }
    };

    // =============================== IMGUI DATA ======================================================================

    auto imguiContext = std::make_unique<ImguiContext>(device_, window, renderGraph.swapChain_->getRenderPass());
    auto render_texture_object = renderGraph.getAttachment<RenderGraphTextureAttachment>("render_result");
    auto render_texture_imgui = render_texture_object->getImguiTextures();
    UIManager uiManager(scene_tree);

    // =============================== IMGUI DATA END ==================================================================

    imgui_render_pass_obj->recordFunction = [&](VkCommandBuffer commandBuffer, uint32_t frame_index) {
        ImGui::Begin("Render Result");
        {
            auto wsize = ImGui::GetContentRegionMax();
            ImGui::Image(render_texture_imgui[frame_index], wsize);
        }
        ImGui::End();

        uiManager.render();

        ImGui::ShowDemoWindow();
        ImGui::Render();
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);
    };

    uint32_t currentFrame = 0;
    uint32_t current_image_index = 0;
    while (not glfwWindowShouldClose(window)) {
        glfwPollEvents();

        auto result = renderGraph.swapChain_->acquireNextImage(&current_image_index);

        if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
            throw std::runtime_error("failed to acquire swap chain image!");
        }

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
            vkDeviceWaitIdle(device_.device());
            renderGraph.recreateSwapChain(extent);
        }

        currentFrame = (currentFrame + 1) % 2;
    }

    vkDeviceWaitIdle(device_.device());

    imguiContext.reset();
    SceneTree::VklNodeMeshBuffer<Mesh3D>::free_instance();
}