#include "CommunicationsSystem.h"
#include "ATCTimer.h"
#include <ctime>        // For std::time_t, std::localtime
#include <iomanip>      // For std::put_time
#include <cmath>
#include <sys/dispatch.h>
#include <cstring> // For memcpy

#define COMMS_CHANNEL_NAME "AH_40247851_40228573_Comms"

CommunicationsSystem::CommunicationsSystem() {
    Communications_System = std::thread(&CommunicationsSystem::HandleCommunications, this);
}

CommunicationsSystem::~CommunicationsSystem() {
    if (Communications_System.joinable()) {
        Communications_System.join();
    }
}

void CommunicationsSystem::HandleCommunications() {
   // std::cout << "Communications System started\n";

    name_attach_t* comms_channel = name_attach(NULL, COMMS_CHANNEL_NAME, 0);

    if (comms_channel == NULL) {
        std::cerr << "Failed to create Communications System channel\n";
        return;
    }

    //std::cout << "Communications System listening on channel: " << COMMS_CHANNEL_NAME << "\n";

    while (true) {
        // Receive Message_inter_process
        Message_inter_process msg;
        memset(&msg, 0, sizeof(msg));  // Clear the buffer before receiving

        int rcvid = MsgReceive(comms_channel->chid, &msg, sizeof(msg), NULL);

        if (rcvid == -1) {
            std::cerr << "Error receiving message: " << strerror(errno) << "\n";
            continue;
        }

        // Skip pulses
        if (rcvid == 0) {
            continue;
        }

        // Check if this is an inter-process message
        if (!msg.header) {
            MsgReply(rcvid, 0, NULL, 0);
            continue;
        }

        // Debug
        std::cout << "Communications System received message:\n";
        std::cout << "  Plane ID: " << msg.planeID << "\n";
        std::cout << "  Type: " << static_cast<int>(msg.type) << "\n";
        std::cout << "  Data Size: " << msg.dataSize << "\n";

        // Reply to acknowledge receipt before forwarding so that we allow the operator console to continue
        int reply = 0;
        MsgReply(rcvid, 0, &reply, sizeof(reply));

        // Process the message based on type
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

            case MessageType::EXIT:
                std::cout << "Exit command received\n";
                name_detach(comms_channel, 0);
                return;

            default:
                std::cerr << "Unknown message type received: " << static_cast<int>(msg.type) << "\n";
                break;
        }
    }

    name_detach(comms_channel, 0);
}

void CommunicationsSystem::messageAircraft(const Message_inter_process& msg) {
    // Open channel to the specific aircraft
    std::string plane_channel_name = "AH_40247851_40228573_" + std::to_string(msg.planeID);
    int plane_channel = name_open(plane_channel_name.c_str(), 0);

    if (plane_channel == -1) {
        std::cerr << "Failed to open channel to Plane " << msg.planeID << " (" << plane_channel_name << ")\n";
        std::cerr << "  Error: " << strerror(errno) << "\n";
        std::cerr << "  Plane may have left airspace or channel doesn't exist yet\n";
        return;
    }

    std::cout << "Successfully opened channel to Plane " << msg.planeID << "\n";

    // Send the Message_inter_process directly to the aircraft
    int reply;
    if (MsgSend(plane_channel, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
        std::cerr << "Failed to send message to Plane " << msg.planeID << "\n";
        std::cerr << "  Error: " << strerror(errno) << "\n";
    } else {
        std::cout << "Successfully sent command to Plane " << msg.planeID << "\n";
    }

    name_close(plane_channel);
}
