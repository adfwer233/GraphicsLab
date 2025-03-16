#pragma once

#include <any>
#include <format>
#include <functional>
#include <string>
#include <vector>

namespace GraphicsLabReflection {

struct Parameter {
    std::any param;

    // @todo: support reference
    // std::reference_wrapper<std::any> param_ref;
};

struct Argument {
    std::string name;
    std::any default_value;
    std::function<const std::type_info &()> type_info_func = nullptr;

    // @todo: only support l-value reference
    // bool is_reference;
};

struct GraphicsLabFunctionMeta {
    // @todo: allow return type
    std::vector<Argument> arguments;
};

struct GraphicsLabFunctionParameterPack {
    std::vector<Parameter> parameters;
};

struct GraphicsLabFunction {
    GraphicsLabFunctionMeta meta;

    std::function<void(GraphicsLabFunctionParameterPack)> function_with_parameter;
};

struct MemberFunctionReflection {
    template <typename C, typename R, typename... args>
    static GraphicsLabFunctionMeta createDefaultFunctionMeta(R (C::*func)(GraphicsLabFunctionParameterPack),
                                                             std::tuple<args...> default_value_tuple) {
        GraphicsLabFunctionMeta result;
        int param_index = 0;
        auto default_values = convertTupleToVectorOfAny(default_value_tuple);
        result.arguments = std::vector<Argument>{
            Argument{.name = std::format("param-{}", param_index++),
                     .default_value = default_values[param_index - 1],
                     .type_info_func = []() -> const std::type_info & { return typeid(args); }}...};
        return result;
    }

    template <typename C, typename R, typename... args>
    static GraphicsLabFunctionMeta createFunctionMetaWithName(R (C::*func)(GraphicsLabFunctionParameterPack),
                                                              std::tuple<args...> default_value_tuple,
                                                              std::vector<std::string> names) {
        if (sizeof...(args) != names.size()) {
            return createDefaultFunctionMeta(func, default_value_tuple);
        }
        GraphicsLabFunctionMeta result;
        int param_index = 0;
        auto default_values = convertTupleToVectorOfAny(default_value_tuple);

        result.arguments = std::vector<Argument>{
            Argument{.name = names[param_index++],
                     .default_value = default_values[param_index - 1],
                     .type_info_func = []() -> const std::type_info & { return typeid(args); }}...};
        return result;
    }

    template <typename C, typename R, typename... args>
    static GraphicsLabFunctionMeta createDefaultFunctionMeta(R (C::*func)(args...),
                                                             std::tuple<args...> default_value_tuple) {
        GraphicsLabFunctionMeta result;
        int param_index = 0;
        auto default_values = convertTupleToVectorOfAny(default_value_tuple);
        result.arguments = std::vector<Argument>{
            Argument{.name = std::format("param-{}", param_index++),
                     .default_value = default_values[param_index - 1],
                     .type_info_func = []() -> const std::type_info & { return typeid(args); }}...};
        return result;
    }

    template <typename C, typename R, typename... args>
    static GraphicsLabFunctionMeta createFunctionMetaWithName(R (C::*func)(args...),
                                                              std::tuple<args...> default_value_tuple,
                                                              std::vector<std::string> names) {
        if (sizeof...(args) != names.size()) {
            return createDefaultFunctionMeta(func, default_value_tuple);
        }
        GraphicsLabFunctionMeta result;
        int param_index = 0;
        auto default_values = convertTupleToVectorOfAny(default_value_tuple);

        result.arguments = std::vector<Argument>{
            Argument{.name = names[param_index++],
                     .default_value = default_values[param_index - 1],
                     .type_info_func = []() -> const std::type_info & { return typeid(args); }}...};
        return result;
    }

  private:
    template <typename... Args> static std::vector<std::any> convertTupleToVectorOfAny(std::tuple<Args...> &tuple) {
        std::vector<std::any> vec;
        vec.reserve(sizeof...(Args)); // Reserve space for optimization

        // Use std::apply to unpack the tuple and insert into the vector
        std::apply(
            [&vec](const Args &...args) {
                (vec.emplace_back(args), ...); // Fold expression to add each element
            },
            tuple);

        return vec;
    }
};

} // namespace GraphicsLabReflection