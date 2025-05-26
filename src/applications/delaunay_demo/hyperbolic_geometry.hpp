#pragma once

#include "glm/glm.hpp"

namespace GraphicsLab::Geometry {

/**
 * @brief Poincare Hyperbolic Disk
 */
struct HyperbolicLineSegment {
    glm::vec2 start_;
    glm::vec2 end_;

    glm::vec2 center_;
    float angle_start_;
    float angle_end_;
    float radius_;

    explicit HyperbolicLineSegment(glm::vec2 start, glm::vec2 end): start_(start), end_(end) {
        if (glm::distance(start, end) < 1e-3) {
            straight = true;
        }

        const float cross = start.x * end.y - start.y * end.x;;
        if (cross < 1e-5) {
            straight = true;
        }

        if (not cross) {
            // Compute center and radius of the circle orthogonal to the unit circle through start and end
            glm::vec2 z = (start - end);
            glm::vec2 perp(-z.y, z.x);
            glm::vec2 mid = 0.5f * (start + end);

            // Solve for the center of the circle
            glm::vec2 dir = glm::normalize(perp);
            float t_center = (1.0f - glm::dot(mid, mid)) / (2.0f * glm::dot(mid, dir));
            center_ = mid + t_center * dir;
            radius_ = glm::length(start - center_);

            // Compute angle for interpolation
            angle_start_ = std::atan2(start.y - center_.y, start.x - center_.x);
            angle_end_ = std::atan2(end.y - center_.y, end.x - center_.x);

            // Ensure shortest arc
            if (angle_end_ < angle_start_) angle_end_ += 2.0f * std::numbers::pi();
        }
    }

    glm::vec2 evaluate(const float t) const {
        if (straight) {
            return glm::mix(start_, end_, t);
        }

        float angle = angle_start_ + t * (angle_end_ - angle_start_);
        return center_ + radius_ * glm::vec2(std::cos(angle), std::sin(angle));
    }

    bool straight = false;
};

}