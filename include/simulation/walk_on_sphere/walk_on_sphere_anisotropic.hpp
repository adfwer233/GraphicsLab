#pragma once

#include "geometry/surface/tensor_product_bezier.hpp"

#include "language/meta_programming/multi_dim_array.hpp"

class AnisotropicWalkOnSphere {
  private:
    TensorProductBezierSurface *targetSurface;

    using cache_data_type = MetaProgramming::MultiDimensionalArray<std::tuple<glm::mat2, glm::vec2>, 100, 100>::type;
    std::unique_ptr<cache_data_type> cache;

    double sdf_evaluate(glm::vec2 param);

    double boundary_evaluation(glm::vec2 param);

    double evaluate_internal(glm::vec2 param);

  public:
    double evaluate(glm::vec2 param);

    TensorProductBezierSurface::render_type getMeshModelBuilderWos();

    explicit AnisotropicWalkOnSphere(TensorProductBezierSurface *surface) : targetSurface(surface) {
    }
};