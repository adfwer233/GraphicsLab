#pragma once

#include "graphics_lab/project.hpp"
#include "spdlog/spdlog.h"

#include "explicit_surface_examples.hpp"

#include <geometry/constructor/explicit_surface_constructors.hpp>
#include <geometry/parametric/tessellator.hpp>
#include <geometry/parametric_intersector/surface_surface_intersector.hpp>

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

    void visualize_intersection() {
        std::scoped_lock lock(context.sceneTree->sceneTreeMutex);

        auto surf = GraphicsLab::Geometry::ExplicitSurfaceConstructor::createHyperboloid();
        GraphicsLab::Geometry::Tessellator::tessellate(surf);
        context.sceneTree->addGeometryNode<GraphicsLab::Geometry::ExplicitSurface>(std::move(surf), "test explicit");

        GraphicsLab::Geometry::Torus torus({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0});
        GraphicsLab::Geometry::Tessellator::tessellate(torus);
        context.sceneTree->addGeometryNode<GraphicsLab::Geometry::Torus>(std::move(torus), "test torus");

        auto result = GraphicsLab::Geometry::SurfaceSurfaceIntersector::intersect_all(torus, surf);

        PointCloud3D point_cloud;

        for (auto &trace : result.traces) {
            for (auto &p : trace) {
                point_cloud.vertices.emplace_back(p.position);
            }
        }
        context.sceneTree->addGeometryNode<PointCloud3D>(std::move(point_cloud), "point cloud");
    }

    void visualize_deformed_torus() {
        auto surf = ExplicitSurfaceExample::createDeformedTorus();
        GraphicsLab::Geometry::Tessellator::tessellate(surf, 64, 64);
        context.sceneTree->addGeometryNode<GraphicsLab::Geometry::ExplicitSurface>(std::move(surf), "test explicit");
    }

    void intersection_demo2() {
        auto surf = ExplicitSurfaceExample::createDeformedTorus();
        GraphicsLab::Geometry::Tessellator::tessellate(surf, 64, 64);
        context.sceneTree->addGeometryNode<GraphicsLab::Geometry::ExplicitSurface>(std::move(surf), "test torus");

        auto surf2 = GraphicsLab::Geometry::ExplicitSurfaceConstructor::createHyperboloid(1.6, 2.0, 1.5);
        GraphicsLab::Geometry::Tessellator::tessellate(surf2);
        context.sceneTree->addGeometryNode<GraphicsLab::Geometry::ExplicitSurface>(std::move(surf2),
                                                                                   "test hyperboloid");

        auto result = GraphicsLab::Geometry::SurfaceSurfaceIntersector::intersect_all(surf, surf2);
        for (int i = 0; auto &trace : result.traces) {
            PointCloud3D point_cloud;
            PointCloud2D pcurve1;

            for (auto &p : trace) {
                point_cloud.vertices.emplace_back(p.position);
                pcurve1.vertices.emplace_back(p.param1);
            }

            context.sceneTree->addGeometryNode<PointCloud3D>(std::move(point_cloud), std::format("curve {}", i));
            context.sceneTree->addGeometryNode<PointCloud2D>(std::move(pcurve1), std::format("pcurve {}", i));
            i++;
        }
    }

    ReflectDataType reflect() override {
        auto result = IGraphicsLabProject::reflect();
        result.emplace("tick", TypeErasedValue(&VisualizationProject::tick, this));
        result.emplace("visualize_intersection", TypeErasedValue(&VisualizationProject::visualize_intersection, this));
        result.emplace("visualize_deformed_torus",
                       TypeErasedValue(&VisualizationProject::visualize_deformed_torus, this));
        result.emplace("intersection_demo2", TypeErasedValue(&VisualizationProject::intersection_demo2, this));
        return result;
    }
};