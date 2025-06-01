#pragma once
#include "imgui_internal.h"
#include "language/reflection/reflectors.hpp"

namespace vkl {

struct ImGuiReflection {
    bool show_function_call_dialog = false;
    std::future<void> function_call_result;
    std::vector<std::any> parameters;
    GraphicsLabReflection::GraphicsLabFunctionMeta function_meta;
    std::function<void(std::vector<std::any> &)> function_to_call;

    template<IsStaticReflectedType T>
    void render_static_reflected_properties(T& obj) {
        auto members = obj.staticReflect();
        std::apply([&](auto &&... props) {
            ([&] {
                using PropertyType = std::decay_t<decltype(props)>;
                if constexpr (IsProperty<PropertyType>) {
                    using FieldType = typename PropertyType::value_type;

                    if constexpr (std::same_as<FieldType, glm::vec3>) {
                        ImGui::InputFloat3(std::format("{}", props.name).c_str(), &(obj.*props.member_ptr).x);
                    } else if constexpr (std::same_as<FieldType, glm::vec2>) {
                        ImGui::InputFloat2(std::format("{}", props.name).c_str(), &(obj.*props.member_ptr).x);
                    } else if constexpr (std::same_as<FieldType, glm::vec4>) {
                        ImGui::InputFloat4(std::format("{}", props.name).c_str(), &(obj.*props.member_ptr).x);
                    } else if constexpr (std::same_as<FieldType, float>) {
                        ImGui::InputFloat(std::format("{}", props.name).c_str(), &(obj.*props.member_ptr));
                    } else if constexpr (std::same_as<FieldType, int>) {
                        ImGui::InputInt(std::format("{}", props.name).c_str(), &(obj.*props.member_ptr));
                    } else if constexpr (std::same_as<FieldType, bool>) {
                        ImGui::Checkbox(std::format("{}", props.name).c_str(), &(obj.*props.member_ptr));
                    } else if constexpr (std::same_as<FieldType, std::string>) {
                        static char buffer[256]; // adjust size as needed
                        auto& str = obj.*props.member_ptr;
                        std::strncpy(buffer, str.c_str(), sizeof(buffer));
                        if (ImGui::InputText(std::format("{}", props.name).c_str(), buffer, sizeof(buffer))) {
                            str = buffer;
                        }
                    } else if constexpr (IsStaticReflectedType<FieldType>) {
                        if (ImGui::TreeNode(std::format("{}", props.name).c_str())) {
                            render_static_reflected_properties(obj.*props.member_ptr);
                            ImGui::TreePop();
                        }
                    } else if constexpr (StdVector<FieldType>) {
                        if (ImGui::TreeNode(std::format("{}", props.name).c_str())) {
                            for (int i = 0; auto& item: obj.*props.member_ptr) {
                                if (ImGui::TreeNode(std::format("{}[{}]", props.name, i).c_str())) {
                                    render_static_reflected_properties(item);
                                    ImGui::TreePop();
                                }
                                i++;
                            }
                            ImGui::TreePop();
                        }
                    } else {
                        // fallback or unsupported type
                        ImGui::Text("%s: (unsupported type)", props.name);
                    }
                }
            }(), ...);
        }, members);
    }

    void render_functions(Reflectable* reflectable) {
        for (auto& [name, erased]: reflectable->reflect()) {
            if (erased.call_func != nullptr) {
                if (ImGui::Button(name.c_str())) {
                    erased.call_func();
                }
            } else if (erased.call_func_with_param != nullptr) {
                if (ImGui::Button(name.c_str())) {
                    parameters.clear();

                    for (auto &arg: erased.call_func_with_param_meta->arguments) {
                        parameters.push_back(arg.default_value);
                    }

                    show_function_call_dialog = true;
                    function_to_call = erased.call_func_with_param;

                    function_meta = erased.call_func_with_param_meta.value();
                }
            }
        }

        if (show_function_call_dialog) {
            render_function_call_dialog();
        }
    }

private:
    void render_function_call_dialog() {
        // Center the window
        ImVec2 center = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

        ImGui::Begin("Input Dialog", &show_function_call_dialog,
                     ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse);

        ImGui::Text("Enter your arguments:");

        for (int i = 0; auto &arg : function_meta.arguments) {

            if (arg.type_info_func() == typeid(float)) {
                float& val = std::any_cast<float&>(parameters[i]);
                ImGui::InputFloat(std::format("Param: {} (Float)", arg.name).c_str(), &val);
            } else if (arg.type_info_func() == typeid(int)) {
                int& val = std::any_cast<int&>(parameters[i]);
                ImGui::InputInt(std::format("Param: {} (Int)", arg.name).c_str(), &val);
            } else if (arg.type_info_func() == typeid(bool)) {
                bool& val = std::any_cast<bool&>(parameters[i]);
                ImGui::Checkbox(std::format("Param: {} (Bool)", arg.name).c_str(), &val);
            } else if (arg.type_info_func() == typeid(std::string)) {
                std::string& val = std::any_cast<std::string&>(parameters[i]);

                // ImGui requires a char buffer for editing strings
                static char buffer[256];
                std::strncpy(buffer, val.c_str(), sizeof(buffer));
                buffer[sizeof(buffer) - 1] = '\0';

                if (ImGui::InputText(std::format("Param: {} (String)", arg.name).c_str(), buffer, sizeof(buffer))) {
                    val = std::string(buffer);
                }

            } else if (arg.type_info_func() == typeid(glm::vec3)) {
                glm::vec3& val = std::any_cast<glm::vec3&>(parameters[i]);
                ImGui::InputFloat3(std::format("Param: {} (Vec3)", arg.name).c_str(), &val[0]);
            }

            i++;
        }

        if (ImGui::Button("OK")) {
            show_function_call_dialog = false;

            function_call_result = std::async(std::launch::async, [this]() {
                try {
                    function_to_call(parameters);
                    spdlog::info("Function executed successfully.");
                } catch (const std::exception& e) {
                    spdlog::error("Function threw an exception: {}", e.what());
                } catch (...) {
                    spdlog::error("Function threw an unknown exception.");
                }
            });

            spdlog::info("Function call launched asynchronously.");
        }

        ImGui::SameLine();

        if (ImGui::Button("Cancel")) {
            // Close the dialog
            show_function_call_dialog = false;
        }

        ImGui::End();
    }
};

}