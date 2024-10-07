#pragma once

#include "component.hpp"
#include "controller/controller.hpp"

#include <pybind11/embed.h>

namespace py = pybind11;
using namespace py::literals;

class PythonTerminalComponent : public UIComponent {
    // Constants
    const static int INPUT_BUFFER_SIZE = 256; // Maximum input buffer size

    struct PythonTerminal {
        char input_buffer[INPUT_BUFFER_SIZE]; // Character array for input
        std::vector<std::string> output_log;  // Stores output logs

        PythonTerminal() {
            clear_input(); // Initialize input buffer with empty string
        }

        void clear_input() {
            memset(input_buffer, 0, sizeof(input_buffer)); // Clear the buffer
        }

        void add_output(const std::string &output) {
            output_log.push_back(output);
        }

        void render_log() {
            for (const auto &line : output_log) {
                ImGui::TextWrapped("%s", line.c_str()); // Render each line of log
            }
        }
    };

    /**
     * @todo: make this to a global context
     */
    // Custom Python Output Redirector
    class PythonOutputRedirector {
      public:
        PythonOutputRedirector() {
            py::exec(R"(
            import sys
            class OutputRedirector:
                def __init__(self):
                    self.output = []
                def write(self, message):
                    if message.strip():
                        self.output.append(message)
                def flush(self):
                    pass
                def get_output(self):
                    return '\n'.join(self.output)
                def clear_output(self):
                    self.output.clear()

            sys.stdout = OutputRedirector()
            sys.stderr = sys.stdout
        )");
        }

        std::string get_output() {
            return py::eval("sys.stdout.get_output()").cast<std::string>();
        }

        void clear_output() {
            py::exec("sys.stdout.clear_output()");
        }
    };

    void run_python_command(PythonTerminal &terminal, PythonOutputRedirector &output_redirector) {
        try {
            // Convert the C-style string (input_buffer) to a std::string
            std::string command(terminal.input_buffer);

            // Log the executed command
            terminal.add_output(">>> " + command);

            // Execute the Python command
            py::exec(command);

            // Fetch Python output
            std::string output = output_redirector.get_output();

            // If there's output, add it to the terminal and C++ console
            if (!output.empty()) {
                terminal.add_output(output);
                std::cout << output; // Also print to C++ console
            }

            // Clear Python's output buffer after each command
            output_redirector.clear_output();
        } catch (const std::exception &e) {
            // Log any errors
            terminal.add_output("Error: " + std::string(e.what()));
        }

        terminal.clear_input(); // Clear input after execution
    }

    // Set up the custom output redirector
    PythonOutputRedirector output_redirector;

    // Create a terminal instance
    PythonTerminal terminal;

  public:
    PythonTerminalComponent(SceneTree::VklSceneTree &sceneTree, Controller &controller) : UIComponent(sceneTree) {
    }

    void render() final {
        // Python terminal UI
        ImGui::Begin("Python Terminal");

        // Display output log
        ImGui::Separator();
        ImGui::Text("Output:");
        ImGui::BeginChild("Scrolling", ImVec2(0, -ImGui::GetTextLineHeightWithSpacing()), true);
        terminal.render_log();
        ImGui::EndChild();

        // Input text field for Python commands
        if (ImGui::InputText("Command", terminal.input_buffer, INPUT_BUFFER_SIZE,
                             ImGuiInputTextFlags_EnterReturnsTrue)) {
            // If 'Enter' is pressed, execute the Python command
            if (strlen(terminal.input_buffer) > 0) {
                run_python_command(terminal, output_redirector);
            }

            // Set focus back to the input field after command execution
            ImGui::SetKeyboardFocusHere(-1); // Focus on the next item (InputText field)
        }

        ImGui::End();
    }
};

META_REGISTER_TYPE(MainComponentRegisterTag, PythonTerminalComponent)