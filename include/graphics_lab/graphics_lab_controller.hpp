#pragma once

namespace GraphicsLab {

/**
 * @brief Interface of project controller, project can implement its own controller to interactive with GraphicsLab UI
 */
struct IGraphicsLabProjectController {
    virtual ~IGraphicsLabProjectController() = default;
    virtual void process_keyboard_input(GLFWwindow *window, float deltaTime) = 0;
    virtual void scroll_callback(GLFWwindow *window, double x_offset, double y_offset) = 0;
    virtual void mouse_button_callback(GLFWwindow *window, int button, int state, int mod) = 0;
    virtual void mouse_callback(GLFWwindow *window, double xposIn, double yposIn) = 0;
};

struct EmptyGraphicsLabController : IGraphicsLabProjectController {
    void process_keyboard_input(GLFWwindow *, float) override {
    }
    void scroll_callback(GLFWwindow *, double, double) override {
    }
    void mouse_button_callback(GLFWwindow *, int, int, int) override {
    }
    void mouse_callback(GLFWwindow *, double, double) override {
    }
};

} // namespace GraphicsLab