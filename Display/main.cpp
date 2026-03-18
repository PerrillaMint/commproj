#include "Display.h"
#include <iostream>

int main() {
    std::cout << "ATC Display System Starting\n\n\n";

    Display display;

    if (!display.initialize()) {
        std::cerr << "Display: Failed to initialize. Exiting.\n";
        return EXIT_FAILURE;
    }

    std::cout << "Display: Initialization complete. Starting display...\n\n";

    display.run();

    return EXIT_SUCCESS;
}
