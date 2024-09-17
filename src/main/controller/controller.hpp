#pragma once

#include "vkl/core/vkl_device.hpp"

#include "vkl/scene_tree/vkl_ray_picker.hpp"

#include "ui_states.hpp"

struct Controller {
    static inline Controller *controller = nullptr;

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
        // auto *controller = static_cast<Controller *>(glfwGetWindowUserPointer(window));
        if (controller->sceneTree_.get().active_camera) {
            controller->sceneTree_.get().active_camera->camera.process_mouse_scroll(y_offset);
        }
    }

    static void mouse_button_callback(GLFWwindow *window, int button, int state, int mod) {
        // auto *controller = static_cast<Controller *>(glfwGetWindowUserPointer(window));
        auto &uiState = controller->uiState_;
        if (button == GLFW_MOUSE_BUTTON_MIDDLE and state == GLFW_PRESS) {
            controller->uiState_.isMouseMidPressing = true;
        }

        if (button == GLFW_MOUSE_BUTTON_MIDDLE and state == GLFW_RELEASE) {
            uiState.isMouseMidPressing = false;
            uiState.mouseFlag = false;
        }

        if (button == GLFW_MOUSE_BUTTON_LEFT and state == GLFW_PRESS) {
            controller->rayPicking(uiState.mouseXPos - uiState.scope_min.x, uiState.mouseYPos - uiState.scope_min.y,
                                   uiState.scope_max.x - uiState.scope_min.x,
                                   uiState.scope_max.y - uiState.scope_min.y);

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

    void rayPicking(float mouse_x_pos, float mouse_y_pos, float width, float height) {

        if (sceneTree_.get().active_camera == nullptr)
            return;

        auto &camera = sceneTree_.get().active_camera->camera;

        auto up = camera.camera_up_axis;
        auto right = camera.camera_right_axis * camera.ratio;

        auto base_on_viewport = camera.position + camera.camera_front * 0.1f - up * 0.0414f - right * 0.0414f;
        up = up * 0.0414f / float(height / 2);
        right = right * 0.0414f / float(width / 2);
        base_on_viewport = base_on_viewport + up * float(mouse_y_pos) + right * float(mouse_x_pos);

        SceneTree::Ray ray(camera.position, base_on_viewport - camera.position);

        SceneTree::RayPicker rayTracer(sceneTree_.get(), ray);
        auto picking_result = rayTracer.trace();

        if (picking_result.has_value()) {
            sceneTree_.get().activeNode = picking_result->hitGeometryNode;

            if (auto meshNode = dynamic_cast<SceneTree::GeometryNode<Mesh3D> *>(picking_result->hitGeometryNode)) {
                uiState_.box = meshNode->data.getMeshBox();
                // spdlog::info("box updated {} {}", Reflection::serialize(uiState_.box.min_pos).dump(), Reflection::serialize(uiState_.box.max_pos).dump());
                uiState_.boxMeshRecreated = false;
            }
        }
    }

  private:
    UIState &uiState_;
    std::reference_wrapper<SceneTree::VklSceneTree> sceneTree_;
};