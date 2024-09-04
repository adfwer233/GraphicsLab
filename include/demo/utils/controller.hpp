#pragma once

#include <functional>

#include "vkl/scene/vkl_object.hpp"
#include "vkl/scene_tree/vkl_camera.hpp"

#include "ray_tracer/ray_picker.hpp"

#include "../../../src/demo/ui/ui_manager.hpp"
#include "GLFW/glfw3.h"
#include "vkl/scene/vkl_scene.hpp"

enum DemoWidgets {
    SceneRendering,
    UVRendering,
    BezierEditing
};

class KeyboardCameraController {
  private:
    Camera *camera;
    UIManager *uiManager_;

    bool is_mouse_left_pressing = false;
    bool is_mouse_pressing = false;
    bool mouse_flag = true;

    float last_x = 1024 / 2.0f;
    float last_y = 1024 / 2.0f;

    float mouse_x_pos;
    float mouse_y_pos;

    std::optional<std::reference_wrapper<VklScene>> scene_;

    bool pressing_shift;

    bool in_region;

    static KeyboardCameraController *instance_;

  public:
    static KeyboardCameraController *instance() {
        if (instance_ == nullptr)
            instance_ = new KeyboardCameraController();
        return instance_;
    }

    std::function<void()> actionCallBack;

    std::optional<RayPicker::RayPickingResult> picking_result;

    std::optional<int> bezier_picking_result;

    void setCamera(Camera &t_camera);

    void processInput(GLFWwindow *window, float deltaTime);

    static void scroll_callback(GLFWwindow *window, double x_offset, double y_offset);

    static void mouse_button_callback(GLFWwindow *window, int button, int state, int mod);

    static void mouse_callback(GLFWwindow *window, double xposIn, double yposIn);

    void set_scene(VklScene &scene);

    void setUIManager(UIManager &uiManager);

    glm::vec2 scope_min;

    glm::vec2 scope_max;

    DemoWidgets currentWidgets;
};
