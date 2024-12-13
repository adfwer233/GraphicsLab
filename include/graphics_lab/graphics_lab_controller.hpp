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
    void process_keyboard_input(GLFWwindow *window, float deltaTime) override {
    }
    void scroll_callback(GLFWwindow *window, double x_offset, double y_offset) override {
    }
    void mouse_button_callback(GLFWwindow *window, int button, int state, int mod) override {
    }
    void mouse_callback(GLFWwindow *window, double xpos, double ypos) override {
    }
};

} // namespace GraphicsLab