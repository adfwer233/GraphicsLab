#pragma once

#include <vector>

#include "glm/glm.hpp"

#include "geometry/autodiff/autodiff.hpp"
#include "surface.hpp"

#include "geometry/curve/bezier_curve_2d.hpp"
#include "geometry/loop/path_2d.hpp"
#include "geometry/mesh/mesh.hpp"
/**
 * @brief Tensor Product Bezier Surface with automatic differentiation
 */
class TensorProductBezierSurface : GeometrySurface {
  private:
    /**
     * control points of the tensor product bezier surface
     */
    std::vector<std::vector<std::array<float, 3>>> control_points_;

    std::vector<std::unique_ptr<Path2D>> paths;

  public:
    TensorProductBezierSurface() = default;

    /**
     * boundary curves of the geometry surface
     */
    std::vector<std::unique_ptr<BezierCurve2D>> boundary_curves;

    explicit TensorProductBezierSurface(decltype(control_points_) &&control_pts) : control_points_(control_pts) {
        initializePeriodicBoundary();
        laplacianEvaluator = std::make_unique<LaplacianEvaluator>(*this);
    }

    explicit TensorProductBezierSurface(decltype(control_points_) &&control_pts,
                                        std::vector<std::vector<std::array<float, 2>>> &&boundary_control_points,
                                        std::vector<size_t> path_indices = {})
        : control_points_(control_pts) {
        for (auto vec : boundary_control_points) {
            boundary_curves.push_back(std::move(std::make_unique<BezierCurve2D>(std::move(vec))));
        }

        if (not path_indices.empty()) {
            for (size_t path_start = 0; auto path_end : path_indices) {
                std::vector<std::vector<std::array<float, 2>>> path;
                std::copy(boundary_control_points.begin() + path_start, boundary_control_points.begin() + path_end,
                          std::back_inserter(path));
                paths.push_back(std::move(std::make_unique<Path2D>(std::move(path))));
                path_start = path_end;
            }
        }

        laplacianEvaluator = std::make_unique<LaplacianEvaluator>(*this);
    }

    explicit TensorProductBezierSurface(std::vector<std::vector<glm::vec3>> &&control_pts) {
        for (const auto &item_i : control_pts) {
            auto &target = control_points_.emplace_back();
            for (auto item_j : item_i) {
                target.emplace_back(std::array<float, 3>{item_j.x, item_j.y, item_j.z});
            }
        }

        initializePeriodicBoundary();
        laplacianEvaluator = std::make_unique<LaplacianEvaluator>(*this);
    }
    /**
     * evaluate the position with given parameter.
     * @param param vec2 in the parameter space.
     * @return position in the physical space.
     */
    glm::vec3 evaluate(glm::vec2 param);

    /**
     * evaluate the position, automatic differential version
     * @param param autodiff vec2, parameter
     */
    autodiff_vec3 evaluate_autodiff(autodiff_vec2 &param);

    /**
     * evaluate the metric tensor at given position
     */
    glm::mat2 evaluate_metric_tensor(glm::vec2 param);

    glm::vec3 evaluate_ru(glm::vec2 param);

    /**
     * evaluate the metric tensor at given position, autodiff version
     * @param param
     * @return
     */
    autodiff_mat2 evaluate_metric_tensor_autodiff(autodiff_vec2 &param);

    /**
     * evaluate the determinant of the  metric tensor at given position
     * @param param
     * @return
     */
    double evaluate_det_metric_tensor(glm::vec2 param);

    /**
     * evaluate the determinant of the metric tensor at given position, autodiff version
     * @param param
     * @return
     */
    autodiff::var evaluate_det_metric_tensor_autodiff(autodiff_vec2 &param);

    /**
     * evaluate the inverse metric tensor g^{ij}
     * @param param
     * @return
     */
    glm::mat2 evaluate_inverse_metric_tensor(glm::vec2 param);

    /**
     * evaluate the inverse metric tensor g^{ij}, autodiff version
     * @param param
     * @return
     */
    autodiff_mat2 evaluate_inverse_metric_tensor_autodiff(autodiff_vec2 &param);

    /**
     * evaluate the diffusion coefficients of the laplacian beltrami operator at given position
     * @param param
     * @return
     */
    glm::mat2 evaluate_laplacian_diffusion_coefficients(glm::vec2 param);

    /**
     * evaluate the drift coefficient of the laplacian beltrami operator at given position
     * @param param
     * @return
     */
    glm::vec2 evaluate_laplacian_drift_coefficients(glm::vec2 param);

    struct LaplacianEvaluator {
        autodiff_vec2 param_autodiff;
        autodiff_vec2 drift_term;

        autodiff_mat2 metric;
        autodiff::var det, det_sqrt, a11, a12, a21, a22;

        explicit LaplacianEvaluator(TensorProductBezierSurface &surface) {
            param_autodiff.x() = 0;
            param_autodiff.y() = 0;

            metric = surface.evaluate_metric_tensor_autodiff(param_autodiff);
            det = metric(0, 0) * metric(1, 1) - metric(0, 1) * metric(1, 0);

            det_sqrt = autodiff::reverse::detail::sqrt(det);

            a11 = metric(1, 1) / det_sqrt;
            a12 = -metric(0, 1) / det_sqrt;
            a21 = -metric(1, 0) / det_sqrt;
            a22 = metric(0, 0) / det_sqrt;
        }

        glm::vec2 evaluate(glm::vec2 param) {
            param_autodiff.x().update(param.x);
            param_autodiff.y().update(param.y);
            a11.update();
            a12.update();
            a21.update();
            a22.update();
            det_sqrt.update();

            auto [b1_0] = autodiff::reverse::detail::derivativesx(a11, autodiff::wrt(param_autodiff.x()));
            auto [b1_1] = autodiff::reverse::detail::derivativesx(a12, autodiff::wrt(param_autodiff.y()));
            auto [b2_0] = autodiff::reverse::detail::derivativesx(a21, autodiff::wrt(param_autodiff.x()));
            auto [b2_1] = autodiff::reverse::detail::derivativesx(a22, autodiff::wrt(param_autodiff.y()));

            drift_term.x() = (b1_0 + b1_1) / det_sqrt;
            drift_term.y() = (b2_0 + b2_1) / det_sqrt;
            return {drift_term.x(), drift_term.y()};
        }
    };

    std::unique_ptr<LaplacianEvaluator> laplacianEvaluator;

    /**
     * implementing the RenderableGeometry concept
     */
    struct IsRenderableGeometry {};
    using render_type = MeshModelTemplate<Vertex3D, TriangleIndex>;

    render_type *getMeshModel() {
        if (mesh_model_ptr) {
            return mesh_model_ptr.get();
        }

        auto builder = getMeshModelBuilder();
        mesh_model_ptr = std::make_unique<render_type>(builder);
        return mesh_model_ptr.get();
    }

    render_type getMeshModelBuilder();

    GeometrySurfaceType type() override {
        return GeometrySurfaceType::TensorProductBezier;
    };

    float containment_test(glm::vec2 test_param);

    /**
     * sample a random on the boundary
     * @return the parameter of the sampled point
     */
    glm::vec2 sample_boundary_parameter();

  private:
    std::unique_ptr<render_type> mesh_model_ptr;

    void initializeBoundary();

    void initializePeriodicBoundary();
};

#include "meta_programming/type_register/type_register.hpp"
using namespace MetaProgramming;
META_REGISTER_TYPE(RenderableGeometryTag, TensorProductBezierSurface);