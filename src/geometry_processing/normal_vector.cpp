#include "geometry_processing/map/normal_vector.hpp"

#define NOMINMAX // windows header min max conflicts with std::min / max
#include "igl/per_vertex_normals.h"

NormalVector::NormalVector(VklModel &model) {
    int n = model.geometry->vertices.size();

    V.resize(n, 3);
    for (int i = 0; i < n; i++) {
        V(i, 0) = model.geometry->vertices[i].position.x;
        V(i, 1) = -model.geometry->vertices[i].position.y;
        V(i, 2) = model.geometry->vertices[i].position.z;
    }

    int nf = model.geometry->indices.size();
    F.resize(nf, 3);
    for (int i = 0; i < nf; i++) {
        F(i, 0) = model.geometry->indices[i].i;
        F(i, 1) = model.geometry->indices[i].j;
        F(i, 2) = model.geometry->indices[i].k;
    }
}

Eigen::MatrixXd NormalVector::perform() {
    Eigen::MatrixXd K;

    igl::per_vertex_normals(V, F, K);

    return K;
}