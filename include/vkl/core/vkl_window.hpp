#pragma once

#define GLFW_INCLUDE_VULKAN
#include "GLFW/glfw3.h"

#include <string>

/**
 * \page windowPage GLFW window
 *
 * ## GLFW
 *
 * ## Encapsulation
 */

class VklWindow {
  private:
    uint32_t width_;
    uint32_t height_;

    GLFWwindow *window_{nullptr};

  public:
    VklWindow(int w, int h, std::string name = "Graphics Lab");

    void createWindowSurface(VkInstance instance, VkSurfaceKHR *surface);

    GLFWwindow *getGLFWwindow() const {
        return this->window_;
    }

    [[nodiscard]] bool shouldClose() const {
        return glfwWindowShouldClose(this->window_);
    }

    VkExtent2D getExtent() {
        return {static_cast<uint32_t>(width_), static_cast<uint32_t>(height_)};
    }

    static void framebufferResizeCallback(GLFWwindow *window, int width, int height);

    bool wasWindowResized() {
        return framebufferResized;
    }
    void resetWindowResizedFlag() {
        framebufferResized = false;
    }

    bool framebufferResized = false;
};