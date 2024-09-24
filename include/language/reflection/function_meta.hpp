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
    std::function<const std::type_info &()> type_info_func;

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

    std::function<void()> function_simple;
    std::function<void(GraphicsLabFunctionParameterPack)> function_with_paramter;
};

struct MemberFunctionReflection {
    template<typename C, typename R, typename ...args>
    static GraphicsLabFunctionMeta createDefaultFunctionMeta(R C::*func(GraphicsLabFunctionParameterPack), C* obj) {
        GraphicsLabFunctionMeta result;
        int param_index = 0;
        result.arguments = std::vector<Argument>{(
            Argument{
                .name = std::format("param-{}", param_index++),
                .type_info_func = []() -> const std::type_info & { return typeid(args); }
            }, ...
        )};
        return result;
    }

    template<typename C, typename R, typename ...args>
    static GraphicsLabFunctionMeta createFunctionMetaWithName(R C::*func(GraphicsLabFunctionParameterPack), C* obj, std::vector<std::string>& names) {
        if (sizeof...(args) != names.size()) {
            return createDefaultFunctionMeta(func, obj);
        }
        GraphicsLabFunctionMeta result;
        int param_index = 0;
        result.arguments = std::vector<Argument>{(
            Argument{
                .name = names[param_index++],
                .type_info_func = []() -> const std::type_info & { return typeid(args); }
            }, ...
        )};
        return result;
    }
};

}