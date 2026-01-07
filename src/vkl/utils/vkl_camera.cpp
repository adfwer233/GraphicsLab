#include "vkl/scene_tree/vkl_camera.hpp"

/**
 * @brief get view transformation from camera
 * @return test
 */
glm::mat4 Camera::get_view_transformation() const {
    return glm::lookAt(position, position + camera_front, camera_up_axis);
}

glm::mat4 Camera::get_proj_transformation() const {
    if (projection_mode == CameraProjectionMode::PERSPECTIVE) {
        return glm::perspective(glm::radians(this->zoom), ratio, 0.1f, 1000.0f);
    } else {
        float orthoSize = glm::distance(position + camera_front, camera_target) * std::tan(this->zoom / 2.0f);
        float orthoHeight = orthoSize;
        float orthoWidth = orthoHeight * ratio;
        return glm::ortho(-orthoWidth, orthoWidth, -orthoHeight, orthoHeight, 0.1f, 1000.0f);
    }
}

/**
 * @brief Processes mouse wheel input for camera zoom
 * @param offset Wheel offset (positive = zoom out/scroll up, negative = zoom in/scroll down)
 * @note Uses hyperbolic speed formula: speed naturally decays as distance decreases,
 *       preventing fast movement through the target point.
 *       Formula: speed = base_speed * (distance / (distance + damping))
 *       Properties: When distance >> damping, speed ~ base_speed
 *                   When distance ~ damping, speed ~ base_speed * 0.5
 *                   When distance -> 0, speed -> 0
 */
void Camera::process_mouse_scroll(float offset) {
    // Configuration parameters
    const float base_speed = 1.0f;   // Maximum movement speed (at infinite distance)
    const float min_distance = 0.5f; // Minimum distance from camera to target (prevents passing through)
    const float damping = 2.0f;      // Damping coefficient, controls deceleration sensitivity

    // Calculate direction and distance from camera to target
    glm::vec3 to_target = camera_target - position;
    float current_distance = glm::length(to_target);

    // Ensure distance is within valid range (avoid division by zero or negative values)
    current_distance = glm::max(current_distance, 1e-6f);

    // Hyperbolic speed decay: speed decreases as distance decreases
    // When current_distance >> damping, speed_factor ~ 1
    // When current_distance ~ 0, speed_factor ~ 0
    float speed_factor = current_distance / (current_distance + damping);
    float actual_speed = base_speed * speed_factor;

    // Calculate actual movement amount (offset is typically 1.0)
    float move_amount = offset * actual_speed;

    // Handle zoom movement in perspective projection
    handle_perspective_zoom(move_amount, min_distance);
}

/**
 * @brief Handles zoom movement for perspective projection
 * @param move_amount Movement distance (positive = move away from target, negative = move toward target)
 * @param min_distance Minimum allowed distance threshold
 * @note Camera moves along the line of sight while keeping the target fixed.
 *       Movement is constrained near minimum distance to prevent passing through target.
 */
void Camera::handle_perspective_zoom(float move_amount, float min_distance) {
    // Calculate direction vector from camera position to target point
    glm::vec3 to_target = camera_target - position;
    float current_distance = glm::length(to_target);

    // Prevent zero vector (use camera front direction when camera coincides with target)
    if (glm::length(to_target) < 1e-3f) {
        to_target = camera_front;
        current_distance = 1.0f; // Set to reasonable value
    } else {
        to_target = glm::normalize(to_target);
    }

    // Calculate new distance after movement
    float new_distance = current_distance - move_amount;

    // Apply distance constraints
    if (move_amount > 0) {
        // Positive movement: moving away from target (zoom out), no upper limit
        // Can add maximum distance constraint if needed:
        // float max_distance = 100.0f;
        // new_distance = glm::min(new_distance, max_distance);
    } else {
        // Negative movement: moving toward target (zoom in), prevent passing through
        new_distance = glm::max(new_distance, min_distance);
    }

    // Recalculate camera position
    position = camera_target - to_target * new_distance;
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