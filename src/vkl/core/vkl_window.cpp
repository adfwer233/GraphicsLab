#include "vkl/core/vkl_window.hpp"

#include <stdexcept>

VklWindow::VklWindow(int width, int height, std::string name) : width_(width), height_(height) {
    glfwInit();

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

    this->window_ = glfwCreateWindow(width, height, name.c_str(), nullptr, nullptr);

    glfwSetWindowUserPointer(window_, this);
    glfwSetFramebufferSizeCallback(window_, framebufferResizeCallback);
}

void VklWindow::createWindowSurface(VkInstance instance, VkSurfaceKHR *surface) {
    if (glfwCreateWindowSurface(instance, window_, nullptr, surface) != VK_SUCCESS) {
        throw std::runtime_error("failed to craete window surface");
    }
}

void VklWindow::framebufferResizeCallback(GLFWwindow *window, int width, int height) {
    auto pVklWindow = reinterpret_cast<VklWindow *>(glfwGetWindowUserPointer(window));
    pVklWindow->framebufferResized = true;
    pVklWindow->width_ = width;
    pVklWindow->height_ = height;
}
