#include <iostream>
#include <iomanip>
#include <memory>
#include <pthread.h>
#include <sched.h>
#include "Aircraft.h"
#include "ATCTimer.h"

void* updatePositionThread(void* arg) {
    Aircraft* aircraft = static_cast<Aircraft*>(arg);
    return reinterpret_cast<void*>(aircraft->updatePosition());
}

// Constructor
Aircraft::Aircraft(int id, double x, double y, double z, double sx, double sy, double sz, int t)
    : id(id), posX(x), posY(y), posZ(z), speedX(sx), speedY(sy), speedZ(sz), arrivalTime(t), inAirspace(true) {
    messageId = -1;
    radarConnectionId = -1;
    airspace = {0, 100000, 0, 100000, 15000, 40000};

    // Create aircraft thread with real-time scheduling
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

    struct sched_param param;
    param.sched_priority = 10; // Base priority for aircraft threads
    pthread_attr_setschedparam(&attr, &param);

    if (pthread_create(&threadId, &attr, updatePositionThread, (void*)this) != 0) {
        // Fallback: try without explicit scheduling attributes
        if (pthread_create(&threadId, NULL, updatePositionThread, (void*)this) != 0) {
            std::cerr << "Error: failed to create thread for aircraft " << id << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    pthread_attr_destroy(&attr);
}

Aircraft::~Aircraft() {}

void Aircraft::printInitialAircraftData() const {
    std::cout << std::left << std::setw(5) << id
              << std::setw(5) << arrivalTime
              << std::setw(5) << posX
              << std::setw(5) << posY
              << std::setw(5) << posZ
              << std::setw(5) << speedX
              << std::setw(5) << speedY
              << std::setw(5) << speedZ
              << "\n";
}

void Aircraft::changeHeading(double vx, double vy, double vz) {
    if (vx != 0) speedX = vx;
    if (vy != 0) speedY = vy;
    if (vz != 0) speedZ = vz;
    std::cout << "Aircraft " << id << " heading changed to: VX=" << speedX
              << " VY=" << speedY << " VZ=" << speedZ << "\n";
}

int Aircraft::updatePosition() {
    ATCTimer timer(1, 0);
    int currentTime = 0;

    // Wait until the arrival time has passed
    while (currentTime < arrivalTime) {
        timer.waitTimer();
        ++currentTime;
    }

    // Open channel with Radar
    if ((radarConnectionId = name_open(RADAR_CHANNEL_NAME, 0)) == -1) {
        perror("Error occurred while creating the channel with Radar");
        return EXIT_FAILURE;
    }

    // Send ENTER_AIRSPACE message to Radar
    Message enterMsg = createEnterAirspaceMessage(id);
    if (MsgSend(radarConnectionId, &enterMsg, sizeof(enterMsg), 0, 0) == -1) {
        std::cout << "Failed to send enter message to Radar!\n";
        return EXIT_FAILURE;
    }

    // Create channel for this aircraft to be reachable by Radar and CommunicationsSystem
    std::string channelName = std::string(AIRCRAFT_CHANNEL_PREFIX) + std::to_string(id);
    name_attach_t* planeChannel = name_attach(NULL, channelName.c_str(), 0);

    if (planeChannel == NULL) {
        std::cerr << "Could not attach plane ID: " << channelName << " to channel\n";
        return EXIT_FAILURE;
    }

    std::cout << "Aircraft " << id << " channel created and listening\n";

    // Position update loop
    while (true) {
        posX += speedX;
        posY += speedY;
        posZ += speedZ;

        // Check if the plane is still within airspace boundaries
        if (posX < airspace.lower_x_boundary || posX > airspace.upper_x_boundary ||
            posY < airspace.lower_y_boundary || posY > airspace.upper_y_boundary ||
            posZ < airspace.lower_z_boundary || posZ > airspace.upper_z_boundary) {

            std::cout << "Aircraft " << id << " exiting airspace\n";
            Message exitMsg = createExitAirspaceMessage(id);
            if (MsgSend(radarConnectionId, &exitMsg, sizeof(exitMsg), 0, 0) == -1) {
                std::cout << "Failed to send exit message to Radar!\n";
                return EXIT_FAILURE;
            }
            break;
        }

        // Listen for incoming requests
        char buffer[sizeof(Message_inter_process)];
        int rcvid = MsgReceive(planeChannel->chid, buffer, sizeof(buffer), NULL);

        if (rcvid != -1) {
            Message* baseMsg = reinterpret_cast<Message*>(buffer);
            MessageType msgType = baseMsg->type;
            int typeValue = static_cast<int>(msgType);

            // Valid MessageType enum values are 0-10
            // REQUEST_POSITION (3) is from Radar
            // Types 4-6 are from Communications System (operator commands)
            bool isValidType = (typeValue >= 0 && typeValue <= 10);
            bool isFromCommsSystem = isValidType && (msgType != MessageType::REQUEST_POSITION);

            if (isFromCommsSystem) {
                Message_inter_process* receivedMsg = reinterpret_cast<Message_inter_process*>(buffer);

                std::cout << "Aircraft " << id << " received inter-process message, type: "
                          << static_cast<int>(receivedMsg->type) << "\n";

                switch (receivedMsg->type) {
                    case MessageType::REQUEST_CHANGE_OF_HEADING: {
                        msg_change_heading* headingData = reinterpret_cast<msg_change_heading*>(receivedMsg->data.data());
                        std::cout << "Aircraft " << id << " received heading change command\n";
                        std::cout << "  New velocities: VX=" << headingData->velocityX
                                  << " VY=" << headingData->velocityY
                                  << " VZ=" << headingData->velocityZ << "\n";
                        changeHeading(headingData->velocityX, headingData->velocityY, headingData->velocityZ);
                        MsgReply(rcvid, 0, NULL, 0);
                        break;
                    }

                    case MessageType::REQUEST_CHANGE_POSITION: {
                        msg_change_position* posData = reinterpret_cast<msg_change_position*>(receivedMsg->data.data());
                        std::cout << "Aircraft " << id << " received position change command\n";
                        std::cout << "  New position: X=" << posData->x
                                  << " Y=" << posData->y
                                  << " Z=" << posData->z << "\n";
                        posX = posData->x;
                        posY = posData->y;
                        posZ = posData->z;
                        std::cout << "Aircraft " << id << " position updated\n";
                        MsgReply(rcvid, 0, NULL, 0);
                        break;
                    }

                    case MessageType::REQUEST_CHANGE_ALTITUDE: {
                        msg_change_heading* altitudeData = reinterpret_cast<msg_change_heading*>(receivedMsg->data.data());
                        std::cout << "Aircraft " << id << " received altitude change command\n";
                        std::cout << "  New altitude: Z=" << altitudeData->altitude << "\n";
                        posZ = altitudeData->altitude;
                        std::cout << "Aircraft " << id << " altitude updated to " << posZ << "\n";
                        MsgReply(rcvid, 0, NULL, 0);
                        break;
                    }

                    default:
                        std::cerr << "Aircraft " << id << " received unknown inter-process message type: "
                                  << static_cast<int>(receivedMsg->type) << "\n";
                        MsgReply(rcvid, -1, NULL, 0);
                        break;
                }
            } else {
                // Message from Radar requesting position
                Message* receivedMsg = reinterpret_cast<Message*>(buffer);

                if (receivedMsg->type == MessageType::REQUEST_POSITION) {
                    msg_plane_info positionData = {id, posX, posY, posZ, speedX, speedY, speedZ};
                    Message posUpdateMsg = createPositionUpdateMessage(id, positionData);
                    MsgReply(rcvid, 0, &posUpdateMsg, sizeof(posUpdateMsg));
                }
            }
        }

        timer.waitTimer();
    }

    name_detach(planeChannel, 0);
    pthread_exit(NULL);

    return 0;
}

int Aircraft::getArrivalTime() {
    return arrivalTime;
}

int Aircraft::getID() {
    return id;
}

Message Aircraft::createEnterAirspaceMessage(int planeID) {
    Message msg;
    msg.type = MessageType::ENTER_AIRSPACE;
    msg.planeID = planeID;
    msg.data = NULL;
    return msg;
}

Message Aircraft::createExitAirspaceMessage(int planeID) {
    Message msg;
    msg.type = MessageType::EXIT_AIRSPACE;
    msg.planeID = planeID;
    msg.data = NULL;
    return msg;
}

Message Aircraft::createPositionUpdateMessage(int planeID, const msg_plane_info& info) {
    Message msg;
    msg.type = MessageType::POSITION_UPDATE;
    msg.planeID = planeID;
    msg.data = (void*)&info;
    return msg;
}
