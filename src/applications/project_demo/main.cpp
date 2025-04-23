#include "graphics_lab/application.hpp"
#include "iostream"

#include "argparse/argparse.hpp"

#include "visualization.hpp"

int main(int argc, char *argv[]) {
    argparse::ArgumentParser args;
    args.add_argument("-i", "--input").default_value(std::string("path"));

    try {
        args.parse_args(argc, argv);
    } catch (const std::exception &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << args;
        std::exit(1);
    }

    GraphicsLabApplication app{};

    app.projectFactory = []() {
        return new VisualizationProject();
    };

    if (args.is_used("--input")) {
        app.appOption.load_obj_path = args.get<std::string>("--input");
    }

    try {
        app.run();
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return 0;
}