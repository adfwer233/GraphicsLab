#pragma once

#include "spdlog/sinks/base_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"
#include <memory>
#include <mutex>
#include <string>
#include <vector>

template <typename Mutex> class ImGuiSink : public spdlog::sinks::base_sink<Mutex> {
  public:
    // Function to retrieve the logs
    std::vector<std::string> get_logs() const {
        std::lock_guard<Mutex> lock(this->mutex_);
        return log_buffer; // Return a copy of the log buffer
    }

  protected:
    void sink_it_(const spdlog::details::log_msg &msg) override {
        spdlog::memory_buf_t formatted;
        this->formatter_->format(msg, formatted);

        {
            std::lock_guard<Mutex> lock(this->mutex_);
            log_buffer.push_back(fmt::to_string(formatted));

            // Optionally limit buffer size
            if (log_buffer.size() > max_logs) {
                log_buffer.erase(log_buffer.begin());
            }
        }
    }

    void flush_() override {
    }

  private:
    mutable Mutex mutex_;                // Mutex to protect log_buffer
    std::vector<std::string> log_buffer; // Buffer for log messages
    size_t max_logs = 1000;              // Max number of logs to keep in buffer
};

// Type alias for convenience
using ImGuiLogSink = ImGuiSink<std::mutex>;

// Singleton LogManager class
class LogManager {
  public:
    // Get the singleton instance of the LogManager
    static LogManager &getInstance() {
        static LogManager instance;
        return instance;
    }

    // Get the logs to render in ImGui
    [[nodiscard]] std::vector<std::string> get_logs() const {
        return imgui_sink_->get_logs();
    }

    // Logs through spdlog
    template <typename... Args> void log_info(const char *fmt, const Args &...args) {
        spdlog::info(fmt, args...);
    }

    // Other log levels can be added here similarly (warn, error, etc.)

    void setLogger(std::string name) {
        // Create the logger with both console and ImGui sinks
        auto logger = std::make_shared<spdlog::logger>(
            name, spdlog::sinks_init_list{
                      std::make_shared<spdlog::sinks::stdout_color_sink_mt>(), // Console sink
                      imgui_sink_                                              // ImGui custom sink
                  });

        // Set the logger as the default one
        spdlog::set_default_logger(logger);
    }

  private:
    // Private constructor to implement singleton
    LogManager() {
        // Create the ImGui sink
        imgui_sink_ = std::make_shared<ImGuiLogSink>();

        // Create the logger with both console and ImGui sinks
        auto logger = std::make_shared<spdlog::logger>(
            "Graphics Lab", spdlog::sinks_init_list{
                                std::make_shared<spdlog::sinks::stdout_color_sink_mt>(), // Console sink
                                imgui_sink_                                              // ImGui custom sink
                            });

        // Set the logger as the default one
        spdlog::set_default_logger(logger);
        spdlog::set_level(spdlog::level::info); // Optional: Set the log level
    }

    // Prevent copying and assignment
    LogManager(const LogManager &) = delete;
    LogManager &operator=(const LogManager &) = delete;

    std::shared_ptr<ImGuiLogSink> imgui_sink_;
};