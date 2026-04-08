#include "CommunicationsSystem.h"
#include "ComputerSystem.h"
#include "ATCTimer.h"
#include <ctime>        // For std::time_t, std::localtime
#include <iomanip>      // For std::put_time
#include <cmath>
#include <sys/dispatch.h>
#include <cstring> // For memcpy

#define COMMS_CHANNEL_NAME "AH_UA_40281796_40252823_Comms"

CommunicationsSystem::CommunicationsSystem() {
    Communications_System = std::thread(&CommunicationsSystem::HandleCommunications, this);
}

CommunicationsSystem::~CommunicationsSystem() {
    if (Communications_System.joinable()) {
        Communications_System.join();
    }
}

void CommunicationsSystem::HandleCommunications() {
    name_attach_t* comms_channel = name_attach(NULL, COMMS_CHANNEL_NAME, 0);

    if (comms_channel == NULL) {
        std::cerr << "Failed to create Communications System channel\n";
        return;
    }

    bool active = true;
    while (active) {
        Message_inter_process msg;
        memset(&msg, 0, sizeof(msg));

        int rcvid = MsgReceive(comms_channel->chid, &msg, sizeof(msg), NULL);

        if (rcvid == -1) {
            std::cerr << "Error receiving message: " << strerror(errno) << "\n";
            continue;
        }

        if (rcvid == 0) continue;

        if (!msg.header) {
            MsgReply(rcvid, 0, NULL, 0);
            continue;
        }

        std::cout << "Communications System received message:\n";
        std::cout << "  Plane ID: " << msg.planeID << "\n";
        std::cout << "  Type: " << static_cast<int>(msg.type) << "\n";
        std::cout << "  Data Size: " << msg.dataSize << "\n";

        int reply = 0;
        MsgReply(rcvid, 0, &reply, sizeof(reply));

        switch (msg.type) {
            case MessageType::REQUEST_CHANGE_OF_HEADING:
                std::cout << "Forwarding heading change request to Plane " << msg.planeID << "\n";
                messageAircraft(msg);
                break;

            case MessageType::REQUEST_CHANGE_POSITION:
                std::cout << "Forwarding position change request to Plane " << msg.planeID << "\n";
                messageAircraft(msg);
                break;

            case MessageType::REQUEST_CHANGE_ALTITUDE:
                std::cout << "Forwarding altitude change request to Plane " << msg.planeID << "\n";
                messageAircraft(msg);
                break;

            case MessageType::CHANGE_TIME_CONSTRAINT_COLLISIONS: {
                int newTime = 0;
                std::memcpy(&newTime, msg.data.data(), sizeof(int));
                ComputerSystem::timeConstraintCollisionFreq.store(newTime);
                std::cout << "Collision time constraint updated to " << newTime << " seconds\n";
                break;
            }

            case MessageType::EXIT:
                std::cout << "Exit command received\n";
                active = false;
                break;

            default:
                std::cerr << "Unknown message type: " << static_cast<int>(msg.type) << "\n";
                break;
        }
    }

    name_detach(comms_channel, 0);
}

void CommunicationsSystem::messageAircraft(const Message_inter_process& msg) {
    std::string plane_channel_name = "AH_UA_40281796_40252823_" + std::to_string(msg.planeID);
    int plane_channel = name_open(plane_channel_name.c_str(), 0);

    if (plane_channel == -1) {
        std::cerr << "Failed to open channel to Plane " << msg.planeID << "\n";
        std::cerr << "  Error: " << strerror(errno) << "\n";
        return;
    }

    std::cout << "Successfully opened channel to Plane " << msg.planeID << "\n";

    int reply;
    int status = MsgSend(plane_channel, &msg, sizeof(msg), &reply, sizeof(reply));
    if (status == -1) {
        std::cerr << "Failed to send message to Plane " << msg.planeID << "\n";
        std::cerr << "  Error: " << strerror(errno) << "\n";
    } else {
        std::cout << "Successfully sent command to Plane " << msg.planeID << "\n";
    }

    name_close(plane_channel);
}
