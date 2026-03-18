#include "CommunicationsSystem.h"
#include "ATCTimer.h"
#include <ctime>
#include <iomanip>
#include <cmath>
#include <sys/dispatch.h>
#include <cstring>

CommunicationsSystem::CommunicationsSystem() {
    communicationsThread = std::thread(&CommunicationsSystem::handleCommunications, this);
}

CommunicationsSystem::~CommunicationsSystem() {
    if (communicationsThread.joinable()) {
        communicationsThread.join();
    }
}

void CommunicationsSystem::handleCommunications() {
    name_attach_t* commsChannel = name_attach(NULL, COMMS_CHANNEL_NAME, 0);

    if (commsChannel == NULL) {
        std::cerr << "Failed to create Communications System channel\n";
        return;
    }

    while (true) {
        Message_inter_process msg;
        memset(&msg, 0, sizeof(msg));

        int rcvid = MsgReceive(commsChannel->chid, &msg, sizeof(msg), NULL);

        if (rcvid == -1) {
            std::cerr << "Error receiving message: " << strerror(errno) << "\n";
            continue;
        }

        // Skip pulses
        if (rcvid == 0) {
            continue;
        }

        // Check if this is an inter-process message
        if (!msg.isInterProcess) {
            MsgReply(rcvid, 0, NULL, 0);
            continue;
        }

        std::cout << "Communications System received message:\n";
        std::cout << "  Plane ID: " << msg.planeID << "\n";
        std::cout << "  Type: " << static_cast<int>(msg.type) << "\n";
        std::cout << "  Data Size: " << msg.dataSize << "\n";

        // Reply to acknowledge receipt before forwarding
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

            case MessageType::EXIT:
                std::cout << "Exit command received\n";
                name_detach(commsChannel, 0);
                return;

            default:
                std::cerr << "Unknown message type received: " << static_cast<int>(msg.type) << "\n";
                break;
        }
    }

    name_detach(commsChannel, 0);
}

void CommunicationsSystem::messageAircraft(const Message_inter_process& msg) {
    std::string planeChannelName = std::string(AIRCRAFT_CHANNEL_PREFIX) + std::to_string(msg.planeID);
    int planeConnectionId = name_open(planeChannelName.c_str(), 0);

    if (planeConnectionId == -1) {
        std::cerr << "Failed to open channel to Plane " << msg.planeID << " (" << planeChannelName << ")\n";
        std::cerr << "  Error: " << strerror(errno) << "\n";
        std::cerr << "  Plane may have left airspace or channel doesn't exist yet\n";
        return;
    }

    std::cout << "Successfully opened channel to Plane " << msg.planeID << "\n";

    int reply;
    if (MsgSend(planeConnectionId, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
        std::cerr << "Failed to send message to Plane " << msg.planeID << "\n";
        std::cerr << "  Error: " << strerror(errno) << "\n";
    } else {
        std::cout << "Successfully sent command to Plane " << msg.planeID << "\n";
    }

    name_close(planeConnectionId);
}
