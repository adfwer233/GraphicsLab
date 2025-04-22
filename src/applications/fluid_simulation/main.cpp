#include "application.hpp"
#include "iostream"

#include "argparse/argparse.hpp"

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

    FluidSimulationApplication app{};

    try {
        app.run();
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return 0;
}