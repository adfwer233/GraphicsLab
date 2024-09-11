#include "simulation/walk_on_sphere/walk_on_sphere_anisotropic.hpp"

#include "effolkronium/random.hpp"

#include "numbers"

#include "language/meta_programming/multi_min_max.hpp"

using Random = effolkronium::random_static;

double AnisotropicWalkOnSphere::evaluate(glm::vec2 param) {
    double res = 0;
    const int iter = 1024;
    for (int i = 0; i < iter; i++) {
        res += evaluate_internal(param);
    }
    return res / iter;
}

double AnisotropicWalkOnSphere::sdf_evaluate(glm::vec2 param) {
    double dist = 10;

    const auto t_start = std::chrono::high_resolution_clock::now();

    // for (auto &boundary: targetSurface->boundary_curves) {
    //     auto [d, u] = boundary->projection(param);
    //     if (d < dist) dist = d;
    // }
    dist = MetaProgramming::min(param.x, param.y, 1 - param.x, 1 - param.y,
                                std::abs(glm::length(param - glm::vec2{0.5, 0.5}) - 0.2f));

    const auto t_end = std::chrono::high_resolution_clock::now();
    // std::cout << std::chrono::duration<double, std::milli>(t_end - t_start) << std::endl;

    return dist;
}

double AnisotropicWalkOnSphere::evaluate_internal(glm::vec2 param) {
    double sdf = sdf_evaluate(param);

    if (sdf < 1e-4)
        return boundary_evaluation(param);

    auto current_param = param;

    int count = 0;

    while (glm::length(current_param - param) < sdf - 1e-4) {
        count += 1;

        int idx1 = current_param.x / 0.01 + 1;
        int idx2 = current_param.y / 0.01 + 1;

        idx1 %= 100;
        idx2 %= 100;

        auto [diffusion, drift] = (*cache)[idx1][idx2];

        glm::mat2 sigma;

        sigma[0][0] = diffusion[0][0];
        sigma[0][1] = 0;
        sigma[1][0] = diffusion[0][1];
        sigma[1][1] = std::sqrt(diffusion[0][0] * diffusion[1][1] - diffusion[0][1] * diffusion[1][0]);
        sigma /= std::sqrt(diffusion[0][0] / 2);

        float time_step = 0.01;
        auto b1 = Random::get<std::normal_distribution<>>(0.0, 1.0);
        auto b2 = Random::get<std::normal_distribution<>>(0.0, 1.0);

        glm::vec2 dB(b1, b2);
        dB = dB * std::sqrt(time_step);

        auto delta = drift * time_step + dB * sigma;

        current_param = current_param + delta;

        if (count > 1000)
            break;
    }

    if (count > 1000)
        std::cout << "failed " << count << std::endl;

    auto dir = glm::normalize(current_param - param);

    glm::vec2 r = dir * float(sdf);

    return evaluate_internal(param + r);
}

double AnisotropicWalkOnSphere::boundary_evaluation(glm::vec2 param) {
    double dist = 10;
    float boundary_param = -1;
    size_t idx = 0;

    for (size_t i = 0; auto &boundary : targetSurface->boundary_curves) {
        auto [d, u] = boundary->projection(param);
        if (d < dist) {
            dist = d;
            boundary_param = u;
            idx = i;
        }
        i++;
    }

    auto pos = targetSurface->evaluate(targetSurface->boundary_curves[idx]->evaluate(boundary_param));

    if (idx > 3) {
        return 0;
    }

    return pos.z;
}

TensorProductBezierSurface::render_type AnisotropicWalkOnSphere::getMeshModelBuilderWos() {
    TensorProductBezierSurface::render_type builder;

    constexpr int n = 20;
    constexpr int m = 20;

    float delta_u = 1.0f / n;
    float delta_v = 1.0f / m;

    float minValue = 1000;
    float maxValue = -1000;

    cache = std::make_unique<cache_data_type>();

    for (int i = 0; i < 100; i++) {
        std::cout << "pre compute " << i << std::endl;
        for (int j = 0; j < 100; j++) {
            glm::vec2 param{i * 0.01, j * 0.01};
            auto diffusion = targetSurface->evaluate_laplacian_diffusion_coefficients(param);
            auto drift = targetSurface->evaluate_laplacian_drift_coefficients(param);
            (*cache)[i][j] = std::make_tuple(diffusion, drift);
        }
    }

    builder.vertices.resize((m + 1) * (n + 1));

    // #pragma omp parallel for num_threads(8)
    for (int i = 0; i <= m; i++) {
        std::cout << "test " << i << std::endl;
        for (int j = 0; j <= n; j++) {
            glm::vec2 param{delta_u * i, delta_v * j};
            auto position = targetSurface->evaluate(param);
            decltype(builder.vertices)::value_type vertex;
            vertex.position = position;

            double res = evaluate(param);

            glm::clamp(res, -1.0, 1.0);
            vertex.color = {res, 0.0, 0.0};

            builder.vertices[i * (n + 1) + j] = vertex;
        }
    }

    for (auto &vertex : builder.vertices) {
        minValue = std::min(minValue, vertex.color.x);
        maxValue = std::max(maxValue, vertex.color.x);
    }

    for (auto &vertex : builder.vertices) {
        vertex.color = (vertex.color - glm::vec3{float(-1), 0.0f, 0.0f}) / float(2);
    }

    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            auto idx1 = i * (n + 1) + j;
            auto idx2 = i * (n + 1) + j + 1;
            auto idx3 = (i + 1) * (n + 1) + j;
            auto idx4 = (i + 1) * (n + 1) + j + 1;

            decltype(builder.indices)::value_type primitive_idx1, primitive_idx2;
            primitive_idx1.i = idx1;
            primitive_idx1.j = idx2;
            primitive_idx1.k = idx4;

            primitive_idx2.i = idx1;
            primitive_idx2.j = idx4;
            primitive_idx2.k = idx3;

            builder.indices.push_back(primitive_idx1);
            builder.indices.push_back(primitive_idx2);
        }
    }

    return builder;
}