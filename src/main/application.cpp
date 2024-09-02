#include "application.hpp"

#include "vkl/render_graph/render_graph.hpp"
#include "vkl/scene/vkl_scene.hpp"
#include "vkl/system/render_system/simple_render_system.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_vulkan.h"

Application::~Application() {
}

void Application::run() {
    GLFWwindow *window = window_.getGLFWwindow();

    VklScene scene(device_, {0, 0, 10}, {0, 1, 0});

    VklModel::BuilderFromImmediateData builder;
    builder.vertices = {
            {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0}},
            {{0.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0}},
            {{1.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0}},
            {{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0}},
    };

    scene.addObject(builder);

    RenderGraphDescriptor graphDescriptor;

    auto output_texture = graphDescriptor.attachment<RenderGraphTextureAttachment>("output_image");
    output_texture->isSwapChain = true;
    output_texture->format = VK_FORMAT_R8G8B8A8_SRGB;
    output_texture->width = 1024;
    output_texture->height = 1024;

    auto render_texture = graphDescriptor.attachment<RenderGraphTextureAttachment>("render_result");
    render_texture->isSwapChain = false;
    render_texture->format = VK_FORMAT_R8G8B8A8_SRGB;
    render_texture->width = 1024;
    render_texture->height = 1024;

    auto simple_render_pass = graphDescriptor.pass<RenderGraphRenderPass>("simple_render_pass");
    simple_render_pass->outTextureAttachmentDescriptors.push_back(render_texture);
    simple_render_pass->width = 1024;
    simple_render_pass->height = 1024;

    auto imgui_render_pass = graphDescriptor.pass<RenderGraphRenderPass>("imgui_render_pass");
    imgui_render_pass->outTextureAttachmentDescriptors.push_back(output_texture);
    imgui_render_pass->width = 1024;
    imgui_render_pass->height = 1024;
    imgui_render_pass->is_submit_pass = true;

    RenderGraph renderGraph(device_, {WIDTH, HEIGHT}, &graphDescriptor, 3);

    renderGraph.createLayouts();

    renderGraph.createInstances();

    auto simple_render_pass_obj = renderGraph.getPass<RenderGraphRenderPass>("simple_render_pass");
    auto imgui_render_pass_obj = renderGraph.getPass<RenderGraphRenderPass>("imgui_render_pass");

    imgui_render_pass_obj->is_submit_pass = true;

    auto simple_render_system = simple_render_pass_obj->getRenderSystem<SimpleRenderSystem<>>(
            device_, "simple_render_system",
            {{std::format("{}/first_triangle_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
             {std::format("{}/first_triangle_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

    simple_render_pass_obj->recordFunction = [&](VkCommandBuffer commandBuffer, uint32_t frame_index) {
        FrameInfo<VklModel> frameInfo{
                .frameIndex = static_cast<int>(frame_index) % 2,
                .frameTime = 0,
                .commandBuffer = commandBuffer,
                .camera = scene.camera,
                .model = *scene.objects.front()->models.front(),
        };

        simple_render_system->renderObject(frameInfo);
    };

    // =============================== IMGUI DATA ======================================================================
    VkDescriptorPoolSize pool_sizes[] = {{VK_DESCRIPTOR_TYPE_SAMPLER, 1000},
                                         {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000},
                                         {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000},
                                         {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000},
                                         {VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000},
                                         {VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000},
                                         {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000},
                                         {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000},
                                         {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000},
                                         {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000},
                                         {VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000}};

    VkDescriptorPoolCreateInfo pool_info = {};
    pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    pool_info.maxSets = 1000;
    pool_info.poolSizeCount = (uint32_t)std::size(pool_sizes);
    pool_info.pPoolSizes = pool_sizes;

    VkDescriptorPool imguiPool;
    vkCreateDescriptorPool(device_.device(), &pool_info, nullptr, &imguiPool);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    // ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForVulkan(window, true);
    ImGui_ImplVulkan_InitInfo init_info = {};
    device_.fillImGuiInitInfo(init_info);
    init_info.DescriptorPool = imguiPool;
    init_info.RenderPass = renderGraph.swapChain_->getRenderPass();
    init_info.Subpass = 0;
    init_info.MinImageCount = 2;
    init_info.ImageCount = 2;
    init_info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;

    ImGui_ImplVulkan_Init(&init_info);

    // =============================== IMGUI DATA END ==================================================================

    imgui_render_pass_obj->recordFunction = [&](VkCommandBuffer commandBuffer, uint32_t frame_index) {
        ImGui::ShowDemoWindow();
        ImGui::Render();
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);
    };

    uint32_t currentFrame = 0;

    while (not glfwWindowShouldClose(window)) {
        glfwPollEvents();

        auto result = renderGraph.swapChain_->acquireNextImage(&currentFrame);

        if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
            throw std::runtime_error("failed to acquire swap chain image!");
        }

        ImGui_ImplGlfw_NewFrame();
        ImGui_ImplVulkan_NewFrame();
        ImGui::NewFrame();

        auto render_result = renderGraph.render(renderGraph.commandBuffers[currentFrame], currentFrame);

        if (render_result == VK_ERROR_OUT_OF_DATE_KHR || render_result == VK_SUBOPTIMAL_KHR || window_.wasWindowResized()) {
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

    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    /** destroy imgui descriptor pool*/
    vkDestroyDescriptorPool(device_.device(), imguiPool, nullptr);
}