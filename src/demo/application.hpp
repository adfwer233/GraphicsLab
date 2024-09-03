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
    static constexpr int WIDTH = 2048;
    static constexpr int HEIGHT = 1024 + 512;

    VklWindow window_{WIDTH, HEIGHT};
    VklDevice device_;
    VklRenderer renderer_{window_, device_};

    VklOffscreenRenderer bezierRenderer_{device_, 1024, 1024};
    VklOffscreenRenderer offscreenRenderer_{device_, 1024, 1024};
    VklOffscreenRenderer uvRender_{device_, 1024, 1024};

  public:
    Application()
        : device_(window_) {

          };
    ~Application();

    Application(const Application &) = delete;
    Application &operator=(const Application &) = delete;

    void run();
};