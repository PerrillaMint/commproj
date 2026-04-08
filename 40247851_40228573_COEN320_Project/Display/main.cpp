#include "Display.h"
#include <iostream>
#include <csignal>

// Global display pointer
Display* g_display = nullptr;


int main() {

    std::cout << "ATC Display System Starting\n\n\n";

    // Create Display instance
    Display display;
    g_display = &display;

    // Initialize the display system
    if (!display.initialize()) {
        std::cerr << "Display: Failed to initialize. Exiting.\n";
        return EXIT_FAILURE;
    }

    std::cout << "Display: Initialization complete. Starting display...\n\n";


    display.run();



    return EXIT_SUCCESS;
}
