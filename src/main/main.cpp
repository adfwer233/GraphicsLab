#include "application.hpp"
#include "application_experimental.hpp"
#include "iostream"

int main() {
    ApplicationExperimental app{};

    try {
        app.run();
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return 0;
}