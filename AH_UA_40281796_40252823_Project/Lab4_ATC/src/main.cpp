#include "AirTrafficControl.h"
#include "Radar.h"
#include "ATCTimer.h"
#include <limits.h>
#include <string.h>
#include <unistd.h>

// Global tick counter
uint64_t tick_counter = 0;
std::atomic<bool> running(true);

void timer_tick() {
    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        tick_counter++;
    }
}

// Returns the directory containing the running binary, with trailing slash
// e.g. "/home/root/Lab4_ATC/src/"
static std::string getBinaryDir() {
    char exePath[PATH_MAX] = {0};

    // QNX: /proc/self/exefile contains the full path of the running binary
    int fd = open("/proc/self/exefile", O_RDONLY);
    if (fd != -1) {
        int n = read(fd, exePath, sizeof(exePath) - 1);
        close(fd);
        if (n > 0) {
            exePath[n] = '\0';
            // Strip trailing newline if present
            char* nl = strchr(exePath, '\n');
            if (nl) *nl = '\0';
            // Truncate to directory portion
            char* lastSlash = strrchr(exePath, '/');
            if (lastSlash) {
                *(lastSlash + 1) = '\0';  // keep the trailing slash
                return std::string(exePath);
            }
        }
    }

    // Fallback: try readlink on /proc/self/exe (some QNX versions)
    memset(exePath, 0, sizeof(exePath));
    ssize_t len = readlink("/proc/self/exe", exePath, sizeof(exePath) - 1);
    if (len != -1) {
        exePath[len] = '\0';
        char* lastSlash = strrchr(exePath, '/');
        if (lastSlash) {
            *(lastSlash + 1) = '\0';
            return std::string(exePath);
        }
    }

    // Final fallback: current working directory
    memset(exePath, 0, sizeof(exePath));
    if (getcwd(exePath, sizeof(exePath))) {
        return std::string(exePath) + "/";
    }

    return "./";
}

int main() {
    std::string binDir = getBinaryDir();
    std::string planesFile = binDir + "planes.txt";

    std::cout << "Binary directory: " << binDir << "\n";
    std::cout << "Loading planes from: " << planesFile << "\n";

    AirTrafficControl atc;
    atc.readPlanesFromFile(planesFile);

    Radar radar(tick_counter);

    std::thread timer_thread(timer_tick);

    atc.startPlanes();

    if (atc.areAllPlanesFinished()) {
        std::cout << "Main function received signal that all aircraft are inactive.\n";
        running = false;
        timer_thread.join();
    }

    return 0;
}
