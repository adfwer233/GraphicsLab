#pragma once

#include "vkl/core/vkl_device.hpp"
#include "vkl/core/vkl_offscreen_renderer.hpp"
#include "vkl/core/vkl_renderer.hpp"
#include "vkl/core/vkl_window.hpp"

#ifndef DATA_DIR
#define DATA_DIR "./shader/"
#endif

class Application {
  private:
    static constexpr int WIDTH = 1024 + 768;
    static constexpr int HEIGHT = 1024;

    VklWindow window_{WIDTH, HEIGHT};
    VklDevice device_;

  public:
    Application() : device_(window_) {};
    ~Application();

    Application(const Application &) = delete;
    Application &operator=(const Application &) = delete;

    void run();
};