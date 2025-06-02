#pragma once

#include "argparse/argparse.hpp"
#include "graphics_lab/graphics_lab_context.hpp"
#include "project.hpp"
#include "vkl/imgui/imgui_context.hpp"

struct ApplicationOption {
    std::optional<std::string> load_obj_path = std::nullopt;
};

struct GraphicsLabApplication {
  private:
    static constexpr int WIDTH = 1024 + 768;
    static constexpr int HEIGHT = 1280;

    GraphicsLab::GraphicsLabInternalContext appContext;

  public:
    std::function<IGraphicsLabProject *()> projectFactory = nullptr;

    GraphicsLabApplication() : appContext(WIDTH, HEIGHT) {};
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
    }

    static void set_args(argparse::ArgumentParser &args) {
        args.add_argument("-i", "--input").default_value(std::string("path"));
        args.add_argument("--style").default_value(std::string("Light"));
        args.add_argument("--font_size").default_value(30).scan<'i', int>();
        ;
    }

    ApplicationOption appOption;
    std::string imgui_style;
    void run();
};