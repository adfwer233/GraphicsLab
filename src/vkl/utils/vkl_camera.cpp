#include "vkl/scene_tree/vkl_camera.hpp"

/**
 * @brief get view transformation from camera
 * @return test
 */
glm::mat4 Camera::get_view_transformation() const {
    return glm::lookAt(position, position + camera_front, camera_up_axis);
}

glm::mat4 Camera::get_proj_transformation() const {
    return glm::perspective(glm::radians(this->zoom), ratio, 0.1f, 1000.0f);
}

void Camera::process_mouse_scroll(float offset) {
    position += camera_front * move_speed * offset;
}

void Camera::process_keyboard() {
    update_camera_vectors();
}

void Camera::update_camera_vectors() {
    // calculate the new Front vector
    glm::vec3 dir;
    dir.z = glm::sin(glm::radians(theta)) * glm::cos(glm::radians(phi));
    dir.y = -glm::cos(glm::radians(theta));
    dir.x = glm::sin(glm::radians(theta)) * glm::sin(glm::radians(phi));

    position = camera_target + glm::length(position - camera_target) * dir;
    camera_front = glm::normalize(-dir);

    // also re-calculate the Right and Up vector
    camera_right_axis =
        glm::normalize(glm::cross(camera_front, world_up)); // normalize the vectors, because their length
    // gets closer to 0 the more you look up or
    // down which results in slower movement.
    camera_up_axis = glm::normalize(glm::cross(camera_right_axis, camera_front));
}

void Camera::process_mouse_movement(float x_offset, float y_offset) {
    x_offset *= mouse_sensitivity;
    y_offset *= mouse_sensitivity;

    phi -= x_offset;
    theta += y_offset;

    update_camera_vectors();
}

void Camera::process_mouse_shift_movement(float x_offset, float y_offset) {
    x_offset *= 0.1f * mouse_sensitivity;
    y_offset *= 0.1f * mouse_sensitivity;

    position += -x_offset * camera_right_axis + y_offset * camera_up_axis;
    camera_target += -x_offset * camera_right_axis + y_offset * camera_up_axis;

    update_camera_vectors();
}