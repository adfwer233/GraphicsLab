#pragma once

#include "graphics_lab/project.hpp"
#include "spdlog/spdlog.h"

struct DelaunayDemoProject : IGraphicsLabProject {
    void tick() override {
        spdlog::info("tick in delaunay demo project");
    }

    std::string name() override {
        return "Visualization";
    }

    void afterLoad() override {
        spdlog::info("project loaded");
    }

    ReflectDataType reflect() override;
};