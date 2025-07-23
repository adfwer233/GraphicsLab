#pragma once

#include "geometry/boundary_representation/faceter/cdt_faceter.hpp"
#include "graphics_lab/project.hpp"
#include "spdlog/spdlog.h"

#include "geometry/boundary_representation/faceter/naive_faceter.hpp"
#include "geometry/boundary_representation/test/intersection_test.hpp"
#include "geometry/boundary_representation/test/test_base.hpp"
#include "geometry/boundary_representation/test/trimming_test.hpp"

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

        test_case = std::make_unique<TrimmingTest1>();
        spdlog::set_level(spdlog::level::debug);
        test_case->run_test();

        if (test_case->result != TestBase::TestResult::Success) {
            spdlog::error("test case failed");
        }

        // show all faces
        for (const auto &[name, face] : test_case->faces) {
            auto mesh = CDTFaceter::naive_facet(face, 50, 50);
            context.sceneTree->addGeometryNode<Mesh3D>(std::move(mesh), name);
        }

        // show all curves
        for (const auto &[name, curve] : test_case->param_curve) {
            GraphicsLab::Geometry::Tessellator::tessellate(*curve);
            auto mesh = *curve->mesh;
            context.sceneTree->addGeometryNode<CurveMesh3D>(std::move(mesh), name);
        }
    }

    ReflectDataType reflect() override {
        auto result = IGraphicsLabProject::reflect();
        result.emplace("visualize", TypeErasedValue(&VisualizationProject::visualize, this));
        return result;
    }
};
