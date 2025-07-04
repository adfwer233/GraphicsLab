#include "graphics_lab/application.hpp"
#include "iostream"

#include "argparse/argparse.hpp"

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
    app.run();

    try {
    } catch (const std::exception &e) {
        spdlog::error("application run failed: {}", e.what());
        return EXIT_FAILURE;
    }

    return 0;
}