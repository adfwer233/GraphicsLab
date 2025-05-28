#pragma once
#include "GLFW/glfw3.h"
#include "graphics_lab/graphics_lab_controller.hpp"
#include "uistate.hpp"

struct CustomController : GraphicsLab::IGraphicsLabProjectController {
    explicit CustomController(ProjectUIState &ui_state) : ui_state_(ui_state) {
    }

    void scroll_callback(GLFWwindow *, double x_offset, double y_offset) override {
    }

    void process_keyboard_input(GLFWwindow *, float) override {
    }

    void mouse_button_callback(GLFWwindow *window, int button, int state, int mod) override {
        if (button == GLFW_MOUSE_BUTTON_LEFT and state == GLFW_PRESS) {
            is_mouse_left_pressing = true;
        }

        if (button == GLFW_MOUSE_BUTTON_LEFT and state == GLFW_RELEASE) {
            is_mouse_left_pressing = false;
            mouse_flag = true;
        }
    }

    void mouse_callback(GLFWwindow *window, double xposIn, double yposIn) override {

        if (mouse_flag) {
            last_mouse_x_pos = xposIn;
            last_mouse_y_pos = yposIn;
            mouse_flag = false;
        }

        float x_offset = xposIn - last_mouse_x_pos;
        float y_offset = -(yposIn - last_mouse_y_pos);

        last_mouse_x_pos = xposIn;
        last_mouse_y_pos = yposIn;

        if (is_mouse_left_pressing) {
            auto vec = glm::normalize(glm::vec2{x_offset, y_offset});
            Mobius trans = MobiusConstructor::move_to_origin({-0.01 * vec.x, 0.01 * vec.y});
            ui_state_.trans = trans.compose(ui_state_.trans);
        }
    }

    float last_mouse_x_pos, last_mouse_y_pos;
    bool mouse_flag = false;
    bool is_mouse_left_pressing = false;
    ProjectUIState &ui_state_;
};