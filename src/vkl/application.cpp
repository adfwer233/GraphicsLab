#include "application.hpp"
#include "vkl_descriptor.hpp"
#include "vkl_model.hpp"

#include "system/simple_render_system.hpp"
#include "utils/keyboard_camera_controller.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_vulkan.h"

#include <functional>

static ImGui_ImplVulkanH_Window g_MainWindowData;

Application::~Application() {
}

void Application::run() {

    /** tmp model data */
    const std::vector<VklModel::Vertex> vertexData = {
            {{-0.5f, -0.5f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.1f}, {1.0f, 0.0f}},
            {{0.5f, -0.5f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.1f}, {0.0f, 0.0f}},
            {{0.5f, 0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.1f}, {0.0f, 1.0f}},
            {{-0.5f, 0.5f, 0.0f}, {1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 0.1f}, {1.0f, 1.0f}},

            {{-0.5f, -0.5f, -0.5f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.1f}, {1.0f, 0.0f}},
            {{0.5f, -0.5f, -0.5f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.1f}, {0.0f, 0.0f}},
            {{0.5f, 0.5f, -0.5f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.1f}, {0.0f, 1.0f}},
            {{-0.5f, 0.5f, -0.5f}, {1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 0.1f}, {1.0f, 1.0f}}
    };

    const std::vector<uint32_t> indices = {
            0, 1, 2, 2, 3, 0,
            4, 5, 6, 6, 7, 4
    };

    VklModel::BuilderFromImmediateData builder;
    builder.vertices = vertexData;
    builder.indices = indices;
    builder.texturePaths = {std::format("{}/blending_transparent_window.png", DATA_DIR)};

    VklModel model(device_, builder);

    auto texture = model.textures_[0];
    auto imageInfo = texture->descriptorInfo();
    /** set uniform buffers */

    std::vector<std::unique_ptr<VklBuffer>> uniformBuffers(VklSwapChain::MAX_FRAMES_IN_FLIGHT);
    for (int i = 0; i < uniformBuffers.size(); i++) {
        uniformBuffers[i] = std::make_unique<VklBuffer>(
            device_, sizeof(GlobalUbo), 1, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
        uniformBuffers[i]->map();
    }

    auto globalSetLayout = VklDescriptorSetLayout::Builder(device_)
                               .addBinding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_ALL_GRAPHICS)
                               .addBinding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT)
                               .build();

    auto globalPool = VklDescriptorPool::Builder(device_)
                          .setMaxSets(VklSwapChain::MAX_FRAMES_IN_FLIGHT * 2)
                          .addPoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VklSwapChain::MAX_FRAMES_IN_FLIGHT * 2)
                          .build();

    std::vector<VkDescriptorSet> globalDescriptorSets(VklSwapChain::MAX_FRAMES_IN_FLIGHT);
    for (int i = 0; i < globalDescriptorSets.size(); i++) {
        auto bufferInfo = uniformBuffers[i]->descriptorInfo();
        VklDescriptorWriter(*globalSetLayout, *globalPool)
            .writeBuffer(0, &bufferInfo)
            .writeImage(1, &imageInfo)
            .build(globalDescriptorSets[i]);
    }

    /** set camera */

    Camera camera({0, 0, 3}, {0, 1, 0});

    KeyboardCameraController::setCamera(camera);

    GLFWwindow *window = window_.getGLFWwindow();

    glfwSetCursorPosCallback(window, KeyboardCameraController::mouse_callback);
    glfwSetScrollCallback(window, KeyboardCameraController::scroll_callback);
    glfwSetMouseButtonCallback(window, KeyboardCameraController::mouse_button_callback);

    /** render system */
    SimpleRenderSystem renderSystem(device_, renderer_.getSwapChainRenderPass(),
                                    globalSetLayout->getDescriptorSetLayout());

    float deltaTime = 0, lastFrame = 0;

    VkDescriptorPoolSize pool_sizes[] = { { VK_DESCRIPTOR_TYPE_SAMPLER, 1000 },
                                          { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000 },
                                          { VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000 },
                                          { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000 },
                                          { VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000 },
                                          { VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000 },
                                          { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000 },
                                          { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000 },
                                          { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000 },
                                          { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000 },
                                          { VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000 } };

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
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForVulkan(window, true);
    ImGui_ImplVulkan_InitInfo init_info = {};
    device_.fillImGuiInitInfo(init_info);
    init_info.DescriptorPool = imguiPool;
    init_info.RenderPass = renderer_.getSwapChainRenderPass();
    init_info.Subpass = 0;
    init_info.MinImageCount = 2;
    init_info.ImageCount = 2;
    init_info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;

    ImGui_ImplVulkan_Init(&init_info);

    bool show_demo_window = true;

    while (not window_.shouldClose()) {
        glfwPollEvents();

        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        KeyboardCameraController::processInput(window_.getGLFWwindow(), deltaTime);

        if (auto commandBuffer = renderer_.beginFrame()) {
            int frameIndex = renderer_.getFrameIndex();

            ImGui::Begin("Messages");
            ImGui::SeparatorText("Scene Information");
            ImGui::LabelText("# Triangles", "0");
            ImGui::LabelText("Camera Position", std::format("{:.3f}, {:.3f}, {:.3f}", camera.position.x, camera.position.y, camera.position.z).c_str());
            ImGui::SeparatorText("Performance");
            ImGui::LabelText("FPS", std::format("{:.3f}", 1 / deltaTime).c_str());
            ImGui::End();

            FrameInfo frameInfo {
                    frameIndex,
                    currentFrame,
                    commandBuffer,
                    camera,
                    &globalDescriptorSets[frameIndex],
                    model
            };

            renderer_.beginSwapChainRenderPass(commandBuffer);

            GlobalUbo ubo{};

            ubo.view = camera.get_view_transformation();
            ubo.proj = camera.get_proj_transformation();
            ubo.model = glm::mat4(1.0f);

            uniformBuffers[frameIndex]->writeToBuffer(&ubo);
            uniformBuffers[frameIndex]->flush();

            renderSystem.renderObject(frameInfo);

            /* ImGui Rendering */
            ImGui::Render();
            ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);

            renderer_.endSwapChainRenderPass(commandBuffer);
            renderer_.endFrame();
        }
    }

    vkDeviceWaitIdle(device_.device());

    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    /** destroy imgui descriptor pool*/
    vkDestroyDescriptorPool(device_.device(), imguiPool, nullptr);
}
