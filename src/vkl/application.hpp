#pragma once

#include "vkl_device.hpp"
#include "vkl_renderer.hpp"
#include "vkl_window.hpp"

#ifndef DATA_DIR
#define DATA_DIR "./shader/"
#endif

class Application {
  private:
    static constexpr int WIDTH = 800;
    static constexpr int HEIGHT = 600;

    VklWindow window_{WIDTH, HEIGHT};
    VklDevice device_;
    VklRenderer renderer_{window_, device_};

  public:
    Application()
        : device_(window_){

          };
    ~Application();

    Application(const Application &) = delete;
    Application &operator=(const Application &) = delete;

    void run();
};