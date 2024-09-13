#include "demo/utils/controller.hpp"

#include "vkl/scene/vkl_geometry_model.hpp"
#include "vkl/scene/vkl_scene.hpp"

#include <iostream>

KeyboardCameraController *KeyboardCameraController::instance_ = nullptr;

void KeyboardCameraController::setCamera(Camera &t_camera) {
    camera = &t_camera;
}

void KeyboardCameraController::processInput(GLFWwindow *window, float deltaTime) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    auto callProcessKeyboard = [&](CameraMovement direction, float deltaTime) {
        actionCallBack();
        camera->process_keyboard(direction, deltaTime);
    };

    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
        pressing_shift = true;
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_RELEASE)
        pressing_shift = false;

    auto controller = KeyboardCameraController::instance();

    if (controller->currentWidgets == BezierEditing || controller->currentWidgets == UVRendering) {
        if (glfwGetKey(window, GLFW_KEY_KP_ADD) == GLFW_PRESS) {
            controller->uiManager_->bezier_zoom_in += deltaTime;
        }

        if (glfwGetKey(window, GLFW_KEY_KP_SUBTRACT) == GLFW_PRESS) {
            controller->uiManager_->bezier_zoom_in -= deltaTime;
        }

        if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
            controller->uiManager_->bezier_shift.y += deltaTime;
        }

        if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
            controller->uiManager_->bezier_shift.y -= deltaTime;
        }

        if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
            controller->uiManager_->bezier_shift.x += deltaTime;
        }

        if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
            controller->uiManager_->bezier_shift.x -= deltaTime;
        }
    }
}

void KeyboardCameraController::scroll_callback(GLFWwindow *window, double x_offset, double y_offset) {
    auto controller = instance_;
    controller->camera->process_mouse_scroll(y_offset);
}

void KeyboardCameraController::mouse_button_callback(GLFWwindow *window, int button, int state, int mod) {
    auto controller = instance_;

    auto addBezierControlPoint = [&](float u, float v) {
        if (controller->uiManager_->bezier_editor_curves.empty()) {
            std::vector<std::array<float, 2>> control_points = {{u, v}};
            controller->uiManager_->bezier_editor_curves.push_back(
                std::move(std::make_unique<BezierCurve2D>(std::move(control_points))));
        } else {
            controller->uiManager_->bezier_editor_curves.front()->add_control_point({u, v});

            auto modelBuffer = VklGeometryModelBuffer<BezierCurve2D>::instance();
            auto curveMesh = modelBuffer->getGeometryModel(controller->scene_->get().device_,
                                                           controller->uiManager_->bezier_editor_curves.front().get());

            curveMesh->reallocateMesh();
        }
    };

    if (not controller->in_region)
        return;

    if (button == GLFW_MOUSE_BUTTON_MIDDLE and state == GLFW_PRESS) {
        controller->is_mouse_pressing = true;
    }

    if (button == GLFW_MOUSE_BUTTON_RIGHT and state == GLFW_PRESS) {
        controller->is_mouse_pressing = true;

        if (controller->currentWidgets == DemoWidgets::BezierEditing) {

            if (not controller->uiManager_->bezier_editor_curves.empty()) {
                auto start_pos = controller->uiManager_->bezier_editor_curves.front()->evaluate(0);
                addBezierControlPoint(start_pos.x, start_pos.y);
            }
        }
    }

    if (button == GLFW_MOUSE_BUTTON_LEFT and state == GLFW_PRESS) {
        if (controller->currentWidgets == DemoWidgets::SceneRendering) {
            controller->uiManager_->pickObject(
                controller->mouse_x_pos - controller->scope_min.x, controller->mouse_y_pos - controller->scope_min.y,
                controller->scope_max.x - controller->scope_min.x, controller->scope_max.y - controller->scope_min.y);
        } else if (controller->currentWidgets == DemoWidgets::BezierEditing) {
            float width = controller->scope_max.x - controller->scope_min.x;
            float height = controller->scope_max.y - controller->scope_min.y;
            float u = (controller->mouse_x_pos - controller->scope_min.x) / width;
            float v = (controller->mouse_y_pos - controller->scope_min.y) / height;

            u = 0.5 + (u - 0.5) / controller->uiManager_->bezier_zoom_in;
            v = 0.5 + (v - 0.5) / controller->uiManager_->bezier_zoom_in;

            u -= controller->uiManager_->bezier_shift.x;
            v -= controller->uiManager_->bezier_shift.y;

            constexpr float picking_threshold = 0.05;

            if (controller->pressing_shift) {
                auto control_pts = controller->uiManager_->bezier_editor_curves.front()->control_point_vec2;
                int closest_id = -1;
                float closest_distance = 1000;

                glm::vec2 cur{u, v};

                for (int i = 0; i < control_pts.size(); i++) {
                    if (closest_distance > glm::length(cur - control_pts[i])) {
                        closest_distance = glm::length(cur - control_pts[i]);
                        closest_id = i;
                    }
                }

                if (closest_distance < picking_threshold) {
                    std::cout << "picking " << closest_id << std::endl;
                    controller->bezier_picking_result = closest_id;
                } else {
                    controller->bezier_picking_result.reset();
                }
            } else {
                addBezierControlPoint(u, v);
            }
        }

        controller->is_mouse_left_pressing = true;
    }

    if (button == GLFW_MOUSE_BUTTON_LEFT and state == GLFW_RELEASE) {
        controller->mouse_flag = true;
        controller->is_mouse_left_pressing = false;
    }

    if (button == GLFW_MOUSE_BUTTON_MIDDLE and state == GLFW_RELEASE) {
        controller->actionCallBack();
        controller->mouse_flag = true;
        controller->is_mouse_pressing = false;
    }

    if (button == GLFW_MOUSE_BUTTON_RIGHT and state == GLFW_RELEASE) {
        controller->actionCallBack();
        controller->mouse_flag = true;
        controller->is_mouse_pressing = false;
    }
}

void KeyboardCameraController::mouse_callback(GLFWwindow *window, double xposIn, double yposIn) {
    auto controller = instance_;

    controller->mouse_x_pos = static_cast<float>(xposIn);
    controller->mouse_y_pos = static_cast<float>(yposIn);

    if (xposIn > controller->scope_min.x and xposIn < controller->scope_max.x and yposIn > controller->scope_min.y and
        yposIn < controller->scope_max.y)
        controller->in_region = true;
    else
        controller->in_region = false;

    if (not(controller->is_mouse_pressing or controller->is_mouse_left_pressing))
        return;

    if (controller->mouse_flag) {
        controller->last_x = controller->mouse_x_pos;
        controller->last_y = controller->mouse_y_pos;
        controller->mouse_flag = false;
    }

    float x_offset = controller->mouse_x_pos - controller->last_x;
    float y_offset = controller->last_y - controller->mouse_y_pos; // reversed since y-coordinates go from bottom to top

    controller->last_x = controller->mouse_x_pos;
    controller->last_y = controller->mouse_y_pos;

    if (controller->is_mouse_pressing) {

        if (controller->currentWidgets == DemoWidgets::SceneRendering) {
            if (controller->pressing_shift)
                controller->camera->process_mouse_shift_movement(x_offset, y_offset);
            else
                controller->camera->process_mouse_movement(x_offset, y_offset);
        }
    }

    if (controller->is_mouse_left_pressing) {
        if (controller->currentWidgets == DemoWidgets::BezierEditing) {
            if (controller->bezier_picking_result.has_value()) {
                controller->uiManager_->bezier_editor_curves.front()->update_control_point(
                    controller->bezier_picking_result.value(), {x_offset / 1024, -y_offset / 1024});

                auto modelBuffer = VklGeometryModelBuffer<BezierCurve2D>::instance();
                auto curveMesh = modelBuffer->getGeometryModel(
                    controller->scene_->get().device_, controller->uiManager_->bezier_editor_curves.front().get());

                curveMesh->reallocateMesh();
            }
        }
    }
}

void KeyboardCameraController::set_scene(VklScene &scene) {
    scene_ = scene;
}

void KeyboardCameraController::setUIManager(UIManager &uiManager) {
    uiManager_ = &uiManager;
}
