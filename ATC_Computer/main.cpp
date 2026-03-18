#include "ComputerSystem.h"
#include "OperatorConsole.h"
#include "CommunicationsSystem.h"

int main() {
    ComputerSystem computerSystem;
    OperatorConsole console;
    CommunicationsSystem comms;

    if (computerSystem.startMonitoring()) {
        computerSystem.joinThread();
    } else {
        std::cerr << "Failed to start monitoring." << std::endl;
    }

    std::cout << "Monitoring stopped. Exiting main." << std::endl;

    return 0;
}
