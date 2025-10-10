#pragma once

#include "vkl/core/vkl_device.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_vulkan.h"

class ImguiContext {
    VklDevice &device_;
    VkDescriptorPool imguiPool;

    // Private constructor for singleton pattern
    ImguiContext(VklDevice &device, GLFWwindow *window, VkRenderPass renderPass) : device_(device) {
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
        pool_info.poolSizeCount = static_cast<uint32_t>(std::size(pool_sizes));
        pool_info.pPoolSizes = pool_sizes;

        vkCreateDescriptorPool(device_.device(), &pool_info, nullptr, &imguiPool);

        // Setup Dear ImGui context
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO &io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls

        // Setup Dear ImGui style

        if (style == "Light") {
            ImGui::StyleColorsLight();
        } else if (style == "Dark") {
            ImGui::StyleColorsDark();
        } else {
            ImGui::StyleColorsClassic();
        }

        // Setup Platform/Renderer backends
        ImGui_ImplGlfw_InitForVulkan(window, true);
        ImGui_ImplVulkan_InitInfo init_info = {};
        device_.fillImGuiInitInfo(init_info);
        init_info.DescriptorPool = imguiPool;

        init_info.MinImageCount = 2;
        init_info.ImageCount = 2;

        init_info.PipelineInfoMain.RenderPass = renderPass;
        init_info.PipelineInfoMain.Subpass = 0;
        init_info.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;

        ImGui_ImplVulkan_Init(&init_info);

        if (set_font) {
            io.Fonts->AddFontFromFileTTF("font/segoeui.ttf", 1.0f * font_size);
        }
    }

  public:
    static inline int font_size = 30;
    static inline bool set_font = true;
    static inline std::string style = "Light";

    void cleanContext() {
        ImGui_ImplVulkan_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        vkDestroyDescriptorPool(device_.device(), imguiPool, nullptr);
    }

    // Delete copy constructor and assignment operator to prevent multiple instances
    ImguiContext(const ImguiContext &) = delete;
    ImguiContext &operator=(const ImguiContext &) = delete;

    // Singleton instance retrieval method
    static ImguiContext *getInstance(VklDevice &device, GLFWwindow *window, VkRenderPass renderPass) {
        if (instance_ == nullptr)
            instance_ = new ImguiContext(device, window, renderPass);
        return instance_;
    }

    static ImguiContext *getInstance() {
        return instance_;
    }

    static inline ImguiContext *instance_ = nullptr;
};