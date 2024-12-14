#pragma once

#include "vkl/core/vkl_device.hpp"

#include "vkl/scene_tree/vkl_ray_picker.hpp"

#include "ui_states.hpp"

#ifdef RENDERDOC_DIR
#include "graphics_lab/utils/render_doc.hpp"
#endif

struct ControllerCallbackHandler {
    static inline GraphicsLab::IGraphicsLabProjectController *internal_controller = nullptr;
    static inline GraphicsLab::IGraphicsLabProjectController *project_controller = nullptr;

    static void scroll_callback(GLFWwindow *window, double x_offset, double y_offset) {
        if (internal_controller != nullptr) {
            internal_controller->scroll_callback(window, x_offset, y_offset);
        }

        if (project_controller != nullptr) {
            project_controller->scroll_callback(window, x_offset, y_offset);
        }
    }

    static void mouse_button_callback(GLFWwindow *window, int button, int state, int mod) {
        if (internal_controller != nullptr) {
            internal_controller->mouse_button_callback(window, button, state, mod);
        }
        if (project_controller != nullptr) {
            project_controller->mouse_button_callback(window, button, state, mod);
        }
    }

    static void mouse_callback(GLFWwindow *window, double xposIn, double yposIn) {
        if (internal_controller != nullptr) {
            internal_controller->mouse_callback(window, xposIn, yposIn);
        }
        if (project_controller != nullptr) {
            project_controller->mouse_callback(window, xposIn, yposIn);
        }
    }
};

struct Controller : GraphicsLab::IGraphicsLabProjectController {

    explicit Controller(UIState &uiState, SceneTree::VklSceneTree &sceneTree)
        : uiState_(uiState), sceneTree_(sceneTree) {
    }

    ~Controller() {
        spdlog::critical("controller destructed");
    }

    void process_keyboard_input(GLFWwindow *window, float deltaTime) {
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);

        if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
            uiState_.isPressingShift = true;
        if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_RELEASE)
            uiState_.isPressingShift = false;

        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            uiState_.isPressingS = true;
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_RELEASE)
            uiState_.isPressingS = false;

        if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS)
            uiState_.isPressingG = true;
        if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS)
            uiState_.isPressingG = false;

        if (glfwGetKey(window, GLFW_KEY_KP_1) == GLFW_PRESS) {
            if (sceneTree_.get().active_camera != nullptr) {
                auto &camera = sceneTree_.get().active_camera->camera;

                camera.position = glm::vec3{0, 0, 10};
                camera.camera_up_axis = glm::vec3{0, 1, 0};
                camera.theta = 90.0f;
                camera.phi = 0.0f;
                camera.camera_target = glm::vec3(0.0f);
                camera.zoom = 45;
                camera.update_camera_vectors();
            }
        }

        if (glfwGetKey(window, GLFW_KEY_KP_2) == GLFW_PRESS) {
            if (sceneTree_.get().active_camera != nullptr) {
                auto &camera = sceneTree_.get().active_camera->camera;
                camera.position = glm::vec3{10, 0, 0};
                camera.camera_up_axis = glm::vec3{0, 1, 0};
                camera.theta = 90.0f;
                camera.phi = 90.0f;
                camera.camera_target = glm::vec3(0.0f);
                camera.zoom = 45;
                camera.update_camera_vectors();
            }
        }

        if (glfwGetKey(window, GLFW_KEY_KP_3) == GLFW_PRESS) {
            if (sceneTree_.get().active_camera != nullptr) {
                auto &camera = sceneTree_.get().active_camera->camera;

                camera.position = glm::vec3{0, -10, 0};
                camera.camera_up_axis = glm::vec3{0, 1, 0.003};
                camera.theta = 0.001f;
                camera.phi = 0.0f;
                camera.camera_target = glm::vec3(0.0f);
                camera.zoom = 45;
                camera.update_camera_vectors();
            }
        }

#ifdef RENDERDOC_DIR
        if (glfwGetKey(window, GLFW_KEY_F1) == GLFW_PRESS) {
            GraphicsLab::RenderDocApi::renderdoc_api->TriggerCapture();
            if (!GraphicsLab::RenderDocApi::renderdoc_api->IsTargetControlConnected())
                GraphicsLab::RenderDocApi::renderdoc_api->LaunchReplayUI(1, nullptr);
        }
#endif
    }

    void scroll_callback(GLFWwindow *window, double x_offset, double y_offset) override {
        if (not uiState_.isMouseInRegion)
            return;

        // auto *controller = static_cast<Controller *>(glfwGetWindowUserPointer(window));
        if (sceneTree_.get().active_camera) {
            sceneTree_.get().active_camera->camera.process_mouse_scroll(y_offset);
        }
    }

    void mouse_button_callback(GLFWwindow *window, int button, int state, int mod) {
        if (not uiState_.isMouseInRegion) {
            return;
        }

        // auto *controller = static_cast<Controller *>(glfwGetWindowUserPointer(window));
        auto &uiState = uiState_;
        if (button == GLFW_MOUSE_BUTTON_MIDDLE and state == GLFW_PRESS) {
            uiState_.isMouseMidPressing = true;
        }

        if (button == GLFW_MOUSE_BUTTON_MIDDLE and state == GLFW_RELEASE) {
            uiState.isMouseMidPressing = false;
            uiState.mouseFlag = false;
        }

        if (button == GLFW_MOUSE_BUTTON_LEFT and state == GLFW_PRESS) {
            rayPicking(uiState.mouseXPos - uiState.scope_min["scene_render_result"].x, uiState.mouseYPos - uiState.scope_min["scene_render_result"].y,
                       uiState.scope_max["scene_render_result"].x - uiState.scope_min["scene_render_result"].x, uiState.scope_max["scene_render_result"].y - uiState.scope_min["scene_render_result"].y);

            uiState.isMouseLeftPressing = true;
        }

        if (button == GLFW_MOUSE_BUTTON_LEFT and state == GLFW_RELEASE) {
            uiState.isMouseLeftPressing = false;
            uiState.mouseFlag = true;
        }
    }

    void mouse_callback(GLFWwindow *window, double xposIn, double yposIn) {
        auto *controller = static_cast<Controller *>(glfwGetWindowUserPointer(window));
        auto &uiState = uiState_;

        uiState.mouseXPos = static_cast<float>(xposIn);
        uiState.mouseYPos = static_cast<float>(yposIn);

        if (xposIn > uiState.scope_min["scene_render_result"].x and xposIn < uiState.scope_max["scene_render_result"].x and yposIn > uiState.scope_min["scene_render_result"].y and
            yposIn < uiState.scope_max["scene_render_result"].y) {
            uiState.isMouseInRegion = true;
        } else {
            uiState.isMouseInRegion = false;
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
            if (sceneTree_.get().active_camera) {
                auto &camera = sceneTree_.get().active_camera->camera;

                // translation
                if (uiState.isPressingG) {
                    if (sceneTree_.get().active_camera) {
                        auto camera = sceneTree_.get().active_camera->camera;

                        auto r = camera.camera_right_axis;
                        auto up = camera.camera_up_axis;
                        if (sceneTree_.get().activeNode) {
                            auto geoNode = dynamic_cast<SceneTree::GeometryNode<Mesh3D> *>(sceneTree_.get().activeNode);

                            geoNode->transformation.translation += r * x_offset * 0.1f - up * y_offset * 0.1f;
                        }
                    }
                } else if (uiState.isPressingS) {
                    if (sceneTree_.get().active_camera) {
                        auto &camera = sceneTree_.get().active_camera->camera;
                        float scaling_factor = 1.0f + 0.1f * y_offset;

                        if (sceneTree_.get().activeNode) {
                            auto geoNode = dynamic_cast<SceneTree::GeometryNode<Mesh3D> *>(sceneTree_.get().activeNode);

                            geoNode->transformation.scaling *= glm::vec3(1.0f) * scaling_factor;
                        }
                    }
                } else if (uiState.isPressingShift) {
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
                try {
                    uiState_.box = meshNode->data.getMeshBox();
                } catch (std::exception e) {
                    spdlog::error(e.what());
                }
                // spdlog::info("box updated {} {}", Reflection::serialize(uiState_.box.min_pos).dump(),
                // Reflection::serialize(uiState_.box.max_pos).dump());
                uiState_.boxMeshRecreated = false;
            }
        }
    }

  private:
    UIState &uiState_;
    std::reference_wrapper<SceneTree::VklSceneTree> sceneTree_;
};