#include "graphics_lab/application.hpp"
#include "iostream"

#include "argparse/argparse.hpp"

#include <cpptrace/from_current.hpp>

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

    CPPTRACE_TRY {
        GraphicsLabApplication app(args);
        app.run();
    } CPPTRACE_CATCH(const std::exception& e) {
        std::cerr<<"Exception: "<<e.what()<<std::endl;
        cpptrace::from_current_exception().print();
    }

    return 0;
}