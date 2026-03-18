#include "OperatorConsole.h"
#include <sstream>
#include "ATCTimer.h"
#include <ctime>
#include <iomanip>
#include <cmath>
#include <sys/dispatch.h>
#include <cstring>
#include <chrono>
#include <thread>

OperatorConsole::OperatorConsole() : exitRequested(false) {
    consoleThread = std::thread(&OperatorConsole::handleConsoleInputs, this);
}

OperatorConsole::~OperatorConsole() {
    exitRequested = true;
    if (consoleThread.joinable()) {
        consoleThread.join();
    }
}

void OperatorConsole::handleConsoleInputs() {
    std::cout << "\n Operator Console Started\n";
    std::cout << "Available commands:\n";
    std::cout << "  1. heading <planeID> <velX> <velY> <velZ> - Change plane heading\n";
    std::cout << "  2. position <planeID> <x> <y> <z> - Change plane position\n";
    std::cout << "  3. altitude <planeID> <z> - Change plane altitude\n";
    std::cout << "  4. exit - Exit the console\n\n\n";

    while (!exitRequested) {
        std::cout << "Enter command: ";
        std::string input;
        std::getline(std::cin, input);

        if (input.empty()) continue;

        logCommand(input);

        std::istringstream iss(input);
        std::string command;
        iss >> command;

        if (command == "exit") {
            std::cout << "Exiting Operator Console\n";
            exitRequested = true;
            break;
        }
        else if (command == "heading") {
            int planeID;
            double velX, velY, velZ;

            if (iss >> planeID >> velX >> velY >> velZ) {
                int commsConnectionId = name_open(COMMS_CHANNEL_NAME, 0);

                if (commsConnectionId == -1) {
                    std::cerr << "Failed to open channel to Communications System\n";
                    std::cerr << "  Error: " << strerror(errno) << "\n";
                    continue;
                }

                Message_inter_process msg;
                memset(&msg, 0, sizeof(msg));

                msg.isInterProcess = true;
                msg.type = MessageType::REQUEST_CHANGE_OF_HEADING;
                msg.planeID = planeID;

                msg_change_heading headingData;
                memset(&headingData, 0, sizeof(headingData));
                headingData.id = planeID;
                headingData.velocityX = velX;
                headingData.velocityY = velY;
                headingData.velocityZ = velZ;
                headingData.altitude = 0;

                msg.dataSize = sizeof(msg_change_heading);
                std::memcpy(msg.data.data(), &headingData, sizeof(msg_change_heading));

                int reply;
                if (MsgSend(commsConnectionId, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
                    std::cerr << "Failed to send message to Communications System\n";
                    std::cerr << "  Error: " << strerror(errno) << "\n";
                } else {
                    std::cout << "Heading change command sent for Plane " << planeID << "\n";
                }

                name_close(commsConnectionId);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } else {
                std::cerr << "Invalid heading command format. Usage: heading <planeID> <velX> <velY> <velZ>\n";
            }
        }
        else if (command == "position") {
            int planeID;
            double x, y, z;

            if (iss >> planeID >> x >> y >> z) {
                int commsConnectionId = name_open(COMMS_CHANNEL_NAME, 0);

                if (commsConnectionId == -1) {
                    std::cerr << "Failed to open channel to Communications System\n";
                    std::cerr << "  Error: " << strerror(errno) << "\n";
                    continue;
                }

                Message_inter_process msg;
                memset(&msg, 0, sizeof(msg));

                msg.isInterProcess = true;
                msg.type = MessageType::REQUEST_CHANGE_POSITION;
                msg.planeID = planeID;

                msg_change_position posData;
                memset(&posData, 0, sizeof(posData));
                posData.x = x;
                posData.y = y;
                posData.z = z;

                msg.dataSize = sizeof(msg_change_position);
                std::memcpy(msg.data.data(), &posData, sizeof(msg_change_position));

                int reply;
                if (MsgSend(commsConnectionId, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
                    std::cerr << "Failed to send message to Communications System\n";
                    std::cerr << "  Error: " << strerror(errno) << "\n";
                } else {
                    std::cout << "Position change command sent for Plane " << planeID << "\n";
                }

                name_close(commsConnectionId);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } else {
                std::cerr << "Invalid position command format. Usage: position <planeID> <x> <y> <z>\n";
            }
        }
        else if (command == "altitude") {
            int planeID;
            double z;

            if (iss >> planeID >> z) {
                int commsConnectionId = name_open(COMMS_CHANNEL_NAME, 0);

                if (commsConnectionId == -1) {
                    std::cerr << "Failed to open channel to Communications System\n";
                    std::cerr << "  Error: " << strerror(errno) << "\n";
                    continue;
                }

                Message_inter_process msg;
                memset(&msg, 0, sizeof(msg));

                msg.isInterProcess = true;
                msg.type = MessageType::REQUEST_CHANGE_ALTITUDE;
                msg.planeID = planeID;

                msg_change_heading altitudeData;
                memset(&altitudeData, 0, sizeof(altitudeData));
                altitudeData.id = planeID;
                altitudeData.altitude = z;
                altitudeData.velocityX = 0;
                altitudeData.velocityY = 0;
                altitudeData.velocityZ = 0;

                msg.dataSize = sizeof(msg_change_heading);
                std::memcpy(msg.data.data(), &altitudeData, sizeof(msg_change_heading));

                int reply;
                if (MsgSend(commsConnectionId, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
                    std::cerr << "Failed to send message to Communications System\n";
                    std::cerr << "  Error: " << strerror(errno) << "\n";
                } else {
                    std::cout << "Altitude change command sent for Plane " << planeID << "\n";
                }

                name_close(commsConnectionId);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } else {
                std::cerr << "Invalid altitude command format. Usage: altitude <planeID> <z>\n";
            }
        }
        else {
            std::cerr << "Unknown command: " << command << "\n";
            std::cerr << "Available commands: heading, position, altitude, exit\n";
        }
    }
}

void OperatorConsole::logCommand(const std::string& command) {
    std::cout << "Command received: " << command << std::endl;
}
