#pragma once

#include "geometry/boundary_representation/faceter/cdt_faceter.hpp"
#include "graphics_lab/project.hpp"
#include "spdlog/spdlog.h"

#include "geometry/boundary_representation/faceter/naive_faceter.hpp"
#include "geometry/boundary_representation/test/registered_tests.hpp"

#include <utils/sampler.hpp>

struct VisualizationProject : IGraphicsLabProject {
    void tick() override {
        spdlog::info("tick in visualization project");
    }

    std::string name() override {
        return "Visualization";
    }

    void afterLoad() override {
        spdlog::info("project loaded");
    }

    void visualize() {
        using namespace GraphicsLab::Geometry::BRep;
        std::unique_ptr<TestBase> test_case = nullptr;

        test_case = std::make_unique<CylinderFaceConstructorTest>();
        // spdlog::set_level(spdlog::level::debug);
        test_case->run_test();

        auto update_color = []<typename MeshType>(MeshType &mesh, glm::vec3 color) {
            for (auto &vert : mesh.vertices) {
                vert.color = color;
            }
        };

        if (test_case->result != TestBase::TestResult::Success) {
            spdlog::error("test case failed");
        }

        // show all faces
        for (const auto &[name, face] : test_case->faces) {
            auto mesh = CDTFaceter::naive_facet(face, 50, 50);

            double r = GraphicsLab::Sampler::sampleUniform();
            double g = GraphicsLab::Sampler::sampleUniform();
            double b = GraphicsLab::Sampler::sampleUniform();

            update_color(mesh, {r, g, b});

            context.sceneTree->addGeometryNode<Mesh3D>(std::move(mesh), name);
        }

        // show all curves
        for (const auto &[name, curve] : test_case->param_curve) {
            GraphicsLab::Geometry::Tessellator::tessellate(*curve, 100);
            auto mesh = *curve->mesh;
            context.sceneTree->addGeometryNode<CurveMesh3D>(std::move(mesh), name);
        }

        // show all edges
        for (const auto &[name, edge] : test_case->edges) {
            auto mesh = NaiveFaceter::naive_edge_facet(edge, 100);
            if (name.find("dropped") != std::string::npos) {
                update_color(mesh, {1, 0, 0});
            }
            context.sceneTree->addGeometryNode<CurveMesh3D>(std::move(mesh), name);
        }

        // show all param pcurves
        for (const auto &[name, curve] : test_case->param_pcurve) {
            auto mesh = NaiveFaceter::naive_pcurve_facet(curve, 50);
            context.sceneTree->addGeometryNode<CurveMesh2D>(std::move(mesh), name);
        }
    }

    ReflectDataType reflect() override {
        auto result = IGraphicsLabProject::reflect();
        result.emplace("visualize", TypeErasedValue(&VisualizationProject::visualize, this));
        return result;
    }
};
