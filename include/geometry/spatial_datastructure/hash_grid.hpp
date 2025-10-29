#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "glm/glm.hpp"
#include "spdlog/spdlog.h"

namespace GraphicsLab::Geometry::HashGrid {

struct Ball {
    glm::vec3 center;
    double radius;
};

struct GridIndex {
    int x, y, z;

    bool operator==(const GridIndex& other) const noexcept {
        return x == other.x && y == other.y && z == other.z;
    }
};

// Hash function for GridIndex
struct GridIndexHash {
    std::size_t operator()(const GridIndex& g) const noexcept {
        std::size_t h1 = std::hash<int>{}(g.x);
        std::size_t h2 = std::hash<int>{}(g.y);
        std::size_t h3 = std::hash<int>{}(g.z);
        // combine hashes
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

class HashGrid {
public:
    explicit HashGrid(float cell_size)
        : cell_size_(cell_size), inv_cell_size_(1.0f / cell_size) {}

    void addBall(const Ball& b) {
        int id = static_cast<int>(balls_.size());
        balls_.push_back(b);

        GridIndex minC = cellIndex(b.center - glm::vec3(b.radius));
        GridIndex maxC = cellIndex(b.center + glm::vec3(b.radius));

        for (int i = minC.x; i <= maxC.x; ++i)
            for (int j = minC.y; j <= maxC.y; ++j)
                for (int k = minC.z; k <= maxC.z; ++k)
                    grid_[{i, j, k}].push_back(id);
    }

    std::vector<int> query(const glm::vec3& p) const {
        GridIndex idx = cellIndex(p);
        std::vector<int> result;

        // Optionally, we could also check neighboring cells
        // if cell_size < max radius
        auto it = grid_.find(idx);
        if (it == grid_.end()) return result;

        for (int id : it->second) {
            const Ball& b = balls_[id];
            if (glm::distance(p, b.center) <= b.radius)
                result.push_back(id);
        }
        return result;
    }

    const Ball& getBall(int id) const { return balls_[id]; }

private:
    GridIndex cellIndex(const glm::vec3& p) const {
        return GridIndex{
            static_cast<int>(std::floor(p.x * inv_cell_size_)),
            static_cast<int>(std::floor(p.y * inv_cell_size_)),
            static_cast<int>(std::floor(p.z * inv_cell_size_))
        };
    }

    float cell_size_;
    float inv_cell_size_;
    std::vector<Ball> balls_;
    std::unordered_map<GridIndex, std::vector<int>, GridIndexHash> grid_;
};

}