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


#define COMMS_CHANNEL_NAME "AH_UA_40281796_40252823_Comms"

OperatorConsole::OperatorConsole() : exit(false) {
    // Spec: store operator requests and commands in a log file
    commandLogFile.open("operator_commands.log", std::ios::out | std::ios::app);
    if (!commandLogFile.is_open()) {
        std::cerr << "Warning: Could not open operator command log file\n";
    }

    Operator_Console = std::thread(&OperatorConsole::HandleConsoleInputs, this);
}

OperatorConsole::~OperatorConsole() {
    exit = true;
    if (Operator_Console.joinable()) {
        Operator_Console.join();
    }
    if (commandLogFile.is_open()) {
        commandLogFile.close();
    }
}

void OperatorConsole::HandleConsoleInputs() {
    std::cout << "\n Operator Console Started\n";
    std::cout << "Available commands:\n";
    std::cout << "  1. heading <planeID> <velX> <velY> <velZ> - Change plane heading\n";
    std::cout << "  2. position <planeID> <x> <y> <z> - Change plane position\n";
    std::cout << "  3. altitude <planeID> <z> - Change plane altitude\n";
    std::cout << "  4. settime <seconds> - Change collision prediction time constraint\n";
    std::cout << "  5. exit - Exit the console\n\n";

    auto openCommsChannel = [&]() -> int {
        int ch = name_open(COMMS_CHANNEL_NAME, 0);
        if (ch == -1)
            std::cerr << "Failed to open channel to Communications System: " << strerror(errno) << "\n";
        return ch;
    };

    auto sendMsg = [&](int ch, Message_inter_process& msg) -> bool {
        int reply;
        if (MsgSend(ch, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
            std::cerr << "Failed to send message: " << strerror(errno) << "\n";
            return false;
        }
        return true;
    };

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
            if (!(iss >> planeID >> velX >> velY >> velZ)) {
                std::cerr << "Usage: heading <planeID> <velX> <velY> <velZ>\n";
                continue;
            }

            int ch = openCommsChannel();
            if (ch == -1) continue;

            Message_inter_process msg;
            memset(&msg, 0, sizeof(msg));
            msg.header = true;
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
            std::memcpy(msg.data.data(), &heading_data, sizeof(msg_change_heading));

            if (sendMsg(ch, msg))
                std::cout << "Heading change command sent for Plane " << planeID << "\n";
            name_close(ch);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else if (command == "position") {
            int planeID;
            double x, y, z;
            if (!(iss >> planeID >> x >> y >> z)) {
                std::cerr << "Usage: position <planeID> <x> <y> <z>\n";
                continue;
            }

            int ch = openCommsChannel();
            if (ch == -1) continue;

            Message_inter_process msg;
            memset(&msg, 0, sizeof(msg));
            msg.header = true;
            msg.type = MessageType::REQUEST_CHANGE_POSITION;
            msg.planeID = planeID;

            msg_change_position pos_data;
            memset(&pos_data, 0, sizeof(pos_data));
            pos_data.x = x;
            pos_data.y = y;
            pos_data.z = z;
            msg.dataSize = sizeof(msg_change_position);
            std::memcpy(msg.data.data(), &pos_data, sizeof(msg_change_position));

            if (sendMsg(ch, msg))
                std::cout << "Position change command sent for Plane " << planeID << "\n";
            name_close(ch);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else if (command == "altitude") {
            int planeID;
            double z;
            if (!(iss >> planeID >> z)) {
                std::cerr << "Usage: altitude <planeID> <z>\n";
                continue;
            }

            int ch = openCommsChannel();
            if (ch == -1) continue;

            Message_inter_process msg;
            memset(&msg, 0, sizeof(msg));
            msg.header = true;
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

            if (sendMsg(ch, msg))
                std::cout << "Altitude change command sent for Plane " << planeID << "\n";
            name_close(ch);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else if (command == "settime") {
            int newTime;
            if (!(iss >> newTime) || newTime <= 0) {
                std::cerr << "Usage: settime <seconds> (must be positive)\n";
                continue;
            }

            int ch = openCommsChannel();
            if (ch == -1) continue;

            Message_inter_process msg;
            memset(&msg, 0, sizeof(msg));
            msg.header = true;
            msg.type = MessageType::CHANGE_TIME_CONSTRAINT_COLLISIONS;
            msg.planeID = -1;
            msg.dataSize = sizeof(int);
            std::memcpy(msg.data.data(), &newTime, sizeof(int));

            if (sendMsg(ch, msg))
                std::cout << "Collision time constraint changed to " << newTime << " seconds\n";
            name_close(ch);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else {
            std::cerr << "Unknown command: " << command << "\n";
            std::cerr << "Available commands: heading, position, altitude, settime, exit\n";
        }
    }
}
// Spec: store the operator requests and commands in a log file
void OperatorConsole::logCommand(const std::string& command) {
    std::cout << "Command received: " << command << std::endl;

    if (commandLogFile.is_open()) {
        auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        char timeBuf[64];
        std::strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
        commandLogFile << "[" << timeBuf << "] " << command << "\n";
        commandLogFile.flush();
    }
}
