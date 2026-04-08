#include "OperatorConsole.h"
#include <sstream>
#include "ATCTimer.h"
#include <ctime>        // For std::time_t, std::localtime
#include <iomanip>      // For std::put_time
#include <cmath>
#include <sys/dispatch.h>
#include "Msg_structs.h"
#include <cstring> // For memcpy
#include <chrono>
#include <thread>


#define COMMS_CHANNEL_NAME "AH_40247851_40228573_Comms"

OperatorConsole::OperatorConsole() : exit(false) {

    Operator_Console = std::thread(&OperatorConsole::HandleConsoleInputs, this);
}

OperatorConsole::~OperatorConsole() {
    exit = true;
    if (Operator_Console.joinable()) {
        Operator_Console.join();
    }
}

void OperatorConsole::HandleConsoleInputs() {
    std::cout << "\n Operator Console Started\n";
    std::cout << "Available commands:\n";
    std::cout << "  1. heading <planeID> <velX> <velY> <velZ> - Change plane heading\n";
    std::cout << "  2. position <planeID> <x> <y> <z> - Change plane position\n";
    std::cout << "  3. altitude <planeID> <z> - Change plane altitude\n";
    std::cout << "  4. exit - Exit the console\n\n\n";


    while (!exit) {
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
            exit = true;
            break;
        }
        else if (command == "heading") {
            int planeID;
            double velX, velY, velZ;

            if (iss >> planeID >> velX >> velY >> velZ) {
                // Open channel to Communications System
                int comms_channel = name_open(COMMS_CHANNEL_NAME, 0);

                if (comms_channel == -1) {
                    std::cerr << "Failed to open channel to Communications System\n";
                    std::cerr << "  Error: " << strerror(errno) << "\n";
                    continue;
                }

                // Create and intitialize message for heading change
                Message_inter_process msg;
                memset(&msg, 0, sizeof(msg));  // Zero out entire structure so everything would be intitialized to 0

                msg.header = true;  // Inter-process
                msg.type = MessageType::REQUEST_CHANGE_OF_HEADING;
                msg.planeID = planeID;

                msg_change_heading heading_data;
                memset(&heading_data, 0, sizeof(heading_data));
                heading_data.ID = planeID;
                heading_data.VelocityX = velX;
                heading_data.VelocityY = velY;
                heading_data.VelocityZ = velZ;
                heading_data.altitude = 0;

                msg.dataSize = sizeof(msg_change_heading);
                std::memcpy(msg.data.data(), &heading_data, sizeof(msg_change_heading)); // copy from src to dst to prepare it to be sent

                int reply;
                if (MsgSend(comms_channel, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
                    std::cerr << "Failed to send message to Communications System\n";
                    std::cerr << "  Error: " << strerror(errno) << "\n";
                } else {
                    std::cout << "Heading change command sent for Plane " << planeID << "\n";
                }

                name_close(comms_channel);


                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } else {
                std::cerr << "Invalid heading command format. Usage: heading <planeID> <velX> <velY> <velZ>\n";
            }
        }
        else if (command == "position") {
            int planeID;
            double x, y, z;

            if (iss >> planeID >> x >> y >> z) {
                // Open channel to Communications System
                int comms_channel = name_open(COMMS_CHANNEL_NAME, 0);

                if (comms_channel == -1) {
                    std::cerr << "Failed to open channel to Communications System\n";
                    std::cerr << "  Error: " << strerror(errno) << "\n";
                    continue;
                }


                Message_inter_process msg;
                memset(&msg, 0, sizeof(msg));

                msg.header = true;  // Inter-process
                msg.type = MessageType::REQUEST_CHANGE_POSITION;
                msg.planeID = planeID;

                msg_change_position pos_data;
                memset(&pos_data, 0, sizeof(pos_data));
                pos_data.x = x;
                pos_data.y = y;
                pos_data.z = z;

                msg.dataSize = sizeof(msg_change_position);
                std::memcpy(msg.data.data(), &pos_data, sizeof(msg_change_position));

                int reply;
                if (MsgSend(comms_channel, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
                    std::cerr << "Failed to send message to Communications System\n";
                    std::cerr << "  Error: " << strerror(errno) << "\n";
                } else {
                    std::cout << "Position change command sent for Plane " << planeID << "\n";
                }

                name_close(comms_channel);


                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } else {
                std::cerr << "Invalid position command format. Usage: position <planeID> <x> <y> <z>\n";
            }
        }
        else if (command == "altitude") {
            int planeID;
            double z;

            if (iss >> planeID >> z) {
                // Open channel to Communications System
                int comms_channel = name_open(COMMS_CHANNEL_NAME, 0);

                if (comms_channel == -1) {
                    std::cerr << "Failed to open channel to Communications System\n";
                    std::cerr << "  Error: " << strerror(errno) << "\n";
                    continue;
                }


                Message_inter_process msg;
                memset(&msg, 0, sizeof(msg));

                msg.header = true;  // Inter-process
                msg.type = MessageType::REQUEST_CHANGE_ALTITUDE;
                msg.planeID = planeID;

                msg_change_heading altitude_data;
                memset(&altitude_data, 0, sizeof(altitude_data));
                altitude_data.ID = planeID;
                altitude_data.altitude = z;
                altitude_data.VelocityX = 0;
                altitude_data.VelocityY = 0;
                altitude_data.VelocityZ = 0;

                msg.dataSize = sizeof(msg_change_heading);
                std::memcpy(msg.data.data(), &altitude_data, sizeof(msg_change_heading));

                int reply;
                if (MsgSend(comms_channel, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
                    std::cerr << "Failed to send message to Communications System\n";
                    std::cerr << "  Error: " << strerror(errno) << "\n";
                } else {
                    std::cout << "Altitude change command sent for Plane " << planeID << "\n";
                }

                name_close(comms_channel);


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
