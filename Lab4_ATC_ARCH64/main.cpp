#include "AirTrafficControl.h"
#include "Radar.h"
#include "ATCTimer.h"

// Global tick counter
uint64_t tick_counter = 0;
std::atomic<bool> running(true);

void timer_tick() {
    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        tick_counter++;
    }
}

int main() {
    AirTrafficControl atc;

    atc.readPlanesFromFile(PLANES_INPUT_FILE);

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
