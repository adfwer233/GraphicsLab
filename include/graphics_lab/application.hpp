#pragma once

#include "argparse/argparse.hpp"
#include "graphics_lab/graphics_lab_context.hpp"
#include "project.hpp"
#include "vkl/imgui/imgui_context.hpp"

struct ApplicationOption {
    std::optional<std::string> load_obj_path = std::nullopt;

    // resolution of main render viewport
    std::pair<int, int> render_resolution = {1024, 1024};
};

struct GraphicsLabApplication {
  private:

    // extent of the main window

    static constexpr int WIDTH = 1024 + 768;
    static constexpr int HEIGHT = 1280;

    GraphicsLab::GraphicsLabInternalContext appContext;

  public:
    std::function<IGraphicsLabProject *()> projectFactory = nullptr;

    GraphicsLabApplication() : appContext(WIDTH, HEIGHT) {
        initialize();
    }

    ~GraphicsLabApplication();

    GraphicsLabApplication(const GraphicsLabApplication &) = delete;
    GraphicsLabApplication &operator=(const GraphicsLabApplication &) = delete;

    explicit GraphicsLabApplication(argparse::ArgumentParser &args) : appContext(WIDTH, HEIGHT) {
        if (args.is_used("--input")) {
            appOption.load_obj_path = args.get<std::string>("--input");
        }
        if (args.is_used("--style")) {
            ImguiContext::style = args.get<std::string>("--style");
        }
        if (args.is_used("--font_size")) {
            ImguiContext::font_size = args.get<int>("--font_size");
        }
        if (args.is_used("--resolution_width")) {
            appOption.render_resolution.first = args.get<int>("--resolution_width");
        }
        if (args.is_used("--resolution_height")) {
            appOption.render_resolution.second = args.get<int>("--resolution_height");
        }

        initialize();
    }

    static void set_args(argparse::ArgumentParser &args) {
        args.add_argument("-i", "--input").default_value(std::string("path"));
        args.add_argument("--style").default_value(std::string("Light"));
        args.add_argument("--font_size").default_value(30).scan<'i', int>();
        args.add_argument("--resolution_width").default_value(1024);
        args.add_argument("--resolution_height").default_value(1024);
    }

    ApplicationOption appOption;
    std::string imgui_style;

    void initialize();
    void run();

    friend struct GraphicsLabApplicationExtension;
};