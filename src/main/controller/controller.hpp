#pragma once

#include "vkl/core/vkl_device.hpp"

#include "ui_states.hpp"

struct Controller {
    static inline Controller* controller = nullptr;

    explicit Controller(UIState &uiState, SceneTree::VklSceneTree &sceneTree)
        : uiState_(uiState), sceneTree_(sceneTree) {
    }

    ~Controller() {
        spdlog::critical("controller destructed");
    }

    void processInput(GLFWwindow *window, float deltaTime) {
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);

        if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
            uiState_.isPressingShift = true;
        if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_RELEASE)
            uiState_.isPressingShift = false;
    }

    static void scroll_callback(GLFWwindow *window, double x_offset, double y_offset) {
        auto *controller = static_cast<Controller *>(glfwGetWindowUserPointer(window));
        if (controller->sceneTree_.get().active_camera) {
            controller->sceneTree_.get().active_camera->camera.process_mouse_scroll(y_offset);
        }
    }

    static void mouse_button_callback(GLFWwindow *window, int button, int state, int mod) {
        auto *controller = static_cast<Controller *>(glfwGetWindowUserPointer(window));
        auto &uiState = controller->uiState_;
        if (button == GLFW_MOUSE_BUTTON_MIDDLE and state == GLFW_PRESS) {
            controller->uiState_.isMouseMidPressing = true;
        }

        if (button == GLFW_MOUSE_BUTTON_MIDDLE and state == GLFW_RELEASE) {
            uiState.isMouseMidPressing = false;
            uiState.mouseFlag = false;
        }

        if (button == GLFW_MOUSE_BUTTON_LEFT and state == GLFW_PRESS) {
            uiState.isMouseLeftPressing = true;
        }

        if (button == GLFW_MOUSE_BUTTON_LEFT and state == GLFW_RELEASE) {
            uiState.isMouseLeftPressing = false;
            uiState.mouseFlag = true;
        }
    }

    static void mouse_callback(GLFWwindow *window, double xposIn, double yposIn) {
        // auto *controller = static_cast<Controller *>(glfwGetWindowUserPointer(window));
        auto &uiState = controller->uiState_;

        uiState.mouseXPos = static_cast<float>(xposIn);
        uiState.mouseYPos = static_cast<float>(yposIn);

        if (xposIn > uiState.scope_min.x and xposIn < uiState.scope_max.x and yposIn > uiState.scope_min.y and
            yposIn < uiState.scope_max.y) {
            uiState.isMouseInRegion = true;
        }

        if (not(uiState.isMouseLeftPressing or uiState.isMouseMidPressing))
            return;

        if (uiState.mouseFlag) {
            uiState.lastMouseXPos = uiState.mouseXPos;
            uiState.lastMouseYPos = uiState.mouseYPos;
            uiState.mouseFlag = false;
        }

        float x_offset = uiState.mouseXPos - uiState.lastMouseXPos;
        float y_offset = -(uiState.mouseYPos - uiState.lastMouseYPos);

        uiState.lastMouseXPos = uiState.mouseXPos;
        uiState.lastMouseYPos = uiState.mouseYPos;

        if (uiState.isMouseLeftPressing) {
            if (controller->sceneTree_.get().active_camera) {
                auto &camera = controller->sceneTree_.get().active_camera->camera;
                if (uiState.isPressingShift) {
                    camera.process_mouse_shift_movement(x_offset, y_offset);
                } else {
                    camera.process_mouse_movement(x_offset, y_offset);
                }
            }
        }
    }

  private:
    UIState &uiState_;
    std::reference_wrapper<SceneTree::VklSceneTree> sceneTree_;
};