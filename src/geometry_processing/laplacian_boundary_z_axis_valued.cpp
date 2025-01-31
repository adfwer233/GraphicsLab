#include "geometry_processing/map/laplacian_boundary_z_axis_valued.hpp"

#include <iostream>

#define NOMINMAX
#include "Eigen/Eigen"
#include <Eigen/Sparse>
#include <igl/boundary_facets.h>
#include <igl/colon.h>
#include <igl/cotmatrix.h>
#include <igl/jet.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/readOFF.h>
#include <igl/setdiff.h>
#include <igl/slice.h>
#include <igl/unique.h>

LaplacianBoundaryZAxisValued::LaplacianBoundaryZAxisValued(VklModel &model) {
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

    igl::boundary_facets(F, E);
}

Eigen::VectorXd LaplacianBoundaryZAxisValued::perform() {

    using namespace Eigen;
    using namespace std;

    // Find boundary vertices
    VectorXi b, IA, IC;
    igl::unique(E, b, IA, IC);
    // List of all vertex indices
    VectorXi all, in;
    igl::colon<int>(0, V.rows() - 1, all);
    // List of interior indices
    igl::setdiff(all, b, in, IA);

    // Construct and slice up Laplacian
    SparseMatrix<double> L, L_in_in, L_in_b;
    igl::cotmatrix(V, F, L);
    igl::slice(L, in, in, L_in_in);
    igl::slice(L, in, b, L_in_b);

    // Dirichlet boundary conditions from z-coordinate
    VectorXd Z = V.col(2);
    VectorXd bc = Z(b);

    // Solve PDE
    SimplicialLLT<SparseMatrix<double>> solver(-L_in_in);
    // slice into solution
    Z(in) = solver.solve(L_in_b * bc);

    // Alternative, short hand
    igl::min_quad_with_fixed_data<double> mqwf;
    // Linear term is 0
    VectorXd B = VectorXd::Zero(V.rows(), 1);
    // Empty constraints
    VectorXd Beq;
    SparseMatrix<double> Aeq;
    // Our cotmatrix is _negative_ definite, so flip sign
    igl::min_quad_with_fixed_precompute((-L).eval(), b, Aeq, true, mqwf);
    igl::min_quad_with_fixed_solve(mqwf, B, bc, Beq, Z);

    return Z;
}
