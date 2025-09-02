#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "glm/glm.hpp"

namespace GraphicsLab::Geometry::KDTree {

template <typename T, size_t dim>
concept KDTreePrimitive = requires(T t, glm::vec<dim, float> p) {
    { t.min_pos() } -> std::convertible_to<glm::vec<dim, float>>;
    { t.max_pos() } -> std::convertible_to<glm::vec<dim, float>>;
    { t.distance_to(p) } -> std::convertible_to<float>;
};

template <size_t dim> struct PointPrimitive {
    using PointType = glm::vec<dim, float>;
    PointType position;

    PointType min_pos() const {
        return position;
    }
    PointType max_pos() const {
        return position;
    }

    float distance_to(PointType p) const {
        return glm::distance(p, position);
    }
};

template <size_t dim> struct LineSegmentPrimitive {
    using PointType = glm::vec<dim, float>;
    PointType start, end;

    PointType min_pos() const {
        return glm::min(start, end);
    }

    PointType max_pos() const {
        return glm::max(start, end);
    }

    float distance_to(PointType p) const {
        // point to line segment distance
        PointType v = end - start;
        PointType w = p - start;

        float c1 = glm::dot(w, v);
        if (c1 <= 0.0f)
            return glm::length(p - start);

        float c2 = glm::dot(v, v);
        if (c2 <= c1)
            return glm::length(p - end);

        float b = c1 / c2;
        PointType proj = start + b * v;
        return glm::length(p - proj);
    }
};

template <size_t dim, KDTreePrimitive<dim> T> struct KDTreeNode {
    std::vector<T> data;
    T median;
    KDTreeNode *left = nullptr;
    KDTreeNode *right = nullptr;
};

template <size_t dim, KDTreePrimitive<dim> T> struct KDTree {
    std::vector<T> primitives;
    KDTreeNode<dim, T> *root = nullptr;

    KDTree(const std::vector<T> &t_primitives) : primitives(t_primitives) {
        root = build(primitives, 0);
    }

    // Recursively build the KDTree
    KDTreeNode<dim, T> *build(std::vector<T> &primitives, size_t depth) {
        if (primitives.empty())
            return nullptr;

        size_t axis = depth % dim;
        size_t median = primitives.size() / 2;

        std::vector<T> left, right, overlapping;

        std::sort(primitives.begin(), primitives.end(),
                  [axis](const T &a, const T &b) { return a.min_pos()[axis] < b.min_pos()[axis]; });

        auto mid_pos = (primitives[median].max_pos() + primitives[median].min_pos()) / 2.0f;

        for (auto &p : primitives) {
            if (p.max_pos()[axis] < mid_pos[axis]) {
                left.push_back(p);
            } else if (p.min_pos()[axis] > mid_pos[axis]) {
                right.push_back(p);
            } else {
                overlapping.push_back(p);
            }
        }

        KDTreeNode<dim, T> *node = new KDTreeNode<dim, T>{ overlapping, { mid_pos } };

        node->left = build(left, depth + 1);
        node->right = build(right, depth + 1);

        return node;
    }

    // Nearest neighbor search
    T nearestNeighbor(KDTreeNode<dim, T> *node, const glm::vec<dim, float> &queryPoint, size_t depth = 0) {
        if (!node)
            return T{}; // Return default object of type T

        size_t axis = depth % dim;

        T best = node->median;
        float bestDist = std::numeric_limits<float>::max();

        for (auto p : node->data) {
            if (p.distance_to(queryPoint) < bestDist) {
                bestDist = p.distance_to(queryPoint);
                best = p;
            }
        }

        // Determine which side of the tree to search first
        KDTreeNode<dim, T> *first = nullptr;
        KDTreeNode<dim, T> *second = nullptr;

        float mid_value = ((node->median.max_pos() + node->median.min_pos()) / 2.0f)[axis];

        if (queryPoint[axis] < mid_value) {
            first = node->left;
            second = node->right;
        } else {
            first = node->right;
            second = node->left;
        }

        // Recursively search the first subtree
        T candidate = nearestNeighbor(first, queryPoint, depth + 1);
        if (candidate.distance_to(queryPoint) < bestDist) {
            best = candidate;
            bestDist = best.distance_to(queryPoint);
        }

        // Check if we need to search the second subtree
        float distToPlane = std::abs(queryPoint[axis] - mid_value);
        if (distToPlane < bestDist) {
            candidate = nearestNeighbor(second, queryPoint, depth + 1);
            if (candidate.distance_to(queryPoint) < bestDist) {
                best = candidate;
            }
        }

        return best;
    }
};
} // namespace GraphicsLab::Geometry::KDTree