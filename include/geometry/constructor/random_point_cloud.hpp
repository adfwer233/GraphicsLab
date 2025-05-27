#pragma once

#include "geometry/mesh/mesh.hpp"
#include "utils/sampler.hpp"

namespace GraphicsLab::Geometry {

struct RandomPointCloud {

    static PointCloud2D random_point_cloud_2d(const int n) {
        PointCloud2D result;

        for (int i = 0; i < n; i++) {
            PointCloud2D::vertex_type vertex;
            vertex.position = {Sampler::sampleUniform(-1.0, 1.0), Sampler::sampleUniform(-1.0, 1.0)};

            vertex.color = {1.0, 0.0, 0.0};
            result.vertices.push_back(vertex);
        }

        return result;
    }

    static PointCloud3D random_point_cloud_3d(const int n) {
        PointCloud3D result;

        for (int i = 0; i < n; i++) {
            PointCloud3D::vertex_type vertex;
            vertex.position = {Sampler::sampleUniform(-1.0, 1.0), Sampler::sampleUniform(-1.0, 1.0),
                               Sampler::sampleUniform(-1.0, 1.0)};

            vertex.color = {1.0, 0.0, 0.0};
            result.vertices.push_back(vertex);
        }

        return result;
    }
};

} // namespace GraphicsLab::Geometry