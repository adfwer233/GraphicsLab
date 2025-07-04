#pragma once

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "language/reflection/static_reflector.hpp"

constexpr float default_pitch = 0.0f;
constexpr float default_yaw = -90.0f;

enum CameraMovement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    DOWN,
    UP
};

enum class CameraProjectionMode {
    PERSPECTIVE,
    ORTHOGRAPHIC
};

/**
 * @brief Camera class for opengl
 */
class Camera {
  public:
    /**
     * @brief calculates the front vector from the Camera's (updated) Euler Angles
     */
    void update_camera_vectors();

    /**
     * @brief position of camera in world coordinate
     */
    glm::vec3 position;

    /**
     * @brief position of scene origin in world space
     */
    glm::vec3 camera_target;

    /**
     * @brief up direction of camera (normalized)
     */
    glm::vec3 camera_up_axis;

    /**
     * @brief right axis direction of camera (normalized)
     */
    glm::vec3 camera_right_axis;

    /**
     * @brief front vector of camera
     */
    glm::vec3 camera_front;

    /**
     * @brief world up
     */
    glm::vec3 world_up;

    /**
     * @brief camera zoom
     */
    float zoom;

    /**
     * @brief move speed for interaction
     */
    const float move_speed;

    /**
     * @brief mouse sensitivity
     */
    const float mouse_sensitivity;

    CameraProjectionMode projection_mode = CameraProjectionMode::PERSPECTIVE;

    float ratio = 1.0;

    float theta, phi;

    REFLECT(Property{"position", &Camera::position}, Property{"camera_target", &Camera::camera_target},
            Property{"camera_up_axis", &Camera::camera_up_axis},
            Property{"camera_right_axis", &Camera::camera_right_axis}, Property{"camera_front", &Camera::camera_front},
            Property{"world_up", &Camera::world_up}, Property{"theta", &Camera::theta}, Property{"phi", &Camera::phi})

    Camera(glm::vec3 pos, glm::vec3 up, glm::vec3 target = glm::vec3(0.0f))
        : zoom(45), move_speed(2.5), mouse_sensitivity(0.1f) {
        position = pos;
        camera_up_axis = up;
        world_up = up;
        camera_target = target;
        theta = 90.0f;
        phi = 0.0f;

        update_camera_vectors();
    }

    glm::mat4 get_view_transformation() const;

    glm::mat4 get_proj_transformation() const;

    /**
     * @brief update camera state with an offset given by mouse scroll
     * @param offset
     */
    void process_mouse_scroll(float offset);

    /**
     * @brief update camera state with offsets given by mouse movement
     * @param x_offset
     * @param y_offset
     */
    void process_mouse_movement(float x_offset, float y_offset);

    /**
     * @brief update camera state with offsets given by mouse movement when pressing shift
     * @param x_offset
     * @param y_offset
     */
    void process_mouse_shift_movement(float x_offset, float y_offset);

    /**
     * @brief process keyboard with given direction and delta time
     */
    void process_keyboard();
};