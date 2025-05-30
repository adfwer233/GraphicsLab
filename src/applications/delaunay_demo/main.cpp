#include "graphics_lab/application.hpp"
#include "iostream"

#include "argparse/argparse.hpp"

#include "project.hpp"

int main(int argc, char *argv[]) {
    argparse::ArgumentParser args;
    GraphicsLabApplication::set_args(args);

    try {
        args.parse_args(argc, argv);
    } catch (const std::exception &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << args;
        std::exit(1);
    }

    GraphicsLabApplication app(args);

    app.projectFactory = []() { return new DelaunayDemoProject(); };

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