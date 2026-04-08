#include <iostream>
#include <iomanip>
#include <memory>
#include <pthread.h>
#include "Aircraft.h"
#include "ATCTimer.h"


#define Radar "AH_UA_40281796_40252823_Radar"

void* updatePositionThread(void* arg) {
    Aircraft* aircraft = static_cast<Aircraft*>(arg);
    return reinterpret_cast<void*>(aircraft->updatePosition());
}

// Constructor definition
Aircraft::Aircraft(int id, double x, double y, double z, double sx, double sy, double sz, int t)
    : id(id), posX(x), posY(y), posZ(z), speedX(sx), speedY(sy), speedZ(sz), arrivalTime(t), inAirspace(true) {
    message_id = -1;
    Radar_id = -1;
    airspace = {0, 100000, 0, 100000, 15000, 40000};

    if (pthread_create(&thread_id, NULL, updatePositionThread, (void*)this) != 0) {
        std::cerr << "Error: failed to create thread for aircraft" << id << std::endl;
        exit(EXIT_FAILURE);
    }
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

void Aircraft::changeHeading(double Vx, double Vy, double Vz) {
    if (Vx != 0) speedX = Vx;
    if (Vy != 0) speedY = Vy;
    if (Vz != 0) speedZ = Vz;
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

    // Open channel with radar
    if ((Radar_id = name_open(Radar, 0)) == -1) {
        perror("Error occurred while creating the channel with Radar");
        return EXIT_FAILURE;
    }

    // Send ENTER_AIRSPACE message
    Message enterAirspaceMessage = createEnterAirspaceMessage(id);
    if (MsgSend(Radar_id, &enterAirspaceMessage, sizeof(enterAirspaceMessage), 0, 0) == -1) {
        std::cout << "Failed to send enter message to Radar!\n";
        return EXIT_FAILURE;
    }

    // Create plane-specific channel for radar polling
    std::string id_str = "AH_UA_40281796_40252823_" + std::to_string(id);
    const char* ID = id_str.c_str();
    name_attach_t* Plane_channel = name_attach(NULL, ID, 0);

    if (Plane_channel == NULL) {
        std::cerr << "Could not attach plane ID: " << ID << " to channel\n";
        return EXIT_FAILURE;
    }

    std::cout << "Aircraft " << id << " channel created and listening\n";

    // Start the position update loop
    while (true) {
        // Update position based on velocity
        posX += speedX;
        posY += speedY;
        posZ += speedZ;

        // Check if the plane is still within airspace boundaries
        if (posX < airspace.lower_x_boundary || posX > airspace.upper_x_boundary ||
            posY < airspace.lower_y_boundary || posY > airspace.upper_y_boundary ||
            posZ < airspace.lower_z_boundary || posZ > airspace.upper_z_boundary) {

            std::cout << "Aircraft " << id << " exiting airspace\n";
            Message exitAirspaceMessage = createExitAirspaceMessage(id);
            if (MsgSend(Radar_id, &exitAirspaceMessage, sizeof(exitAirspaceMessage), 0, 0) == -1) {
                std::cout << "Failed to send exit message to Radar!\n";
                return EXIT_FAILURE;
            }
            break;
        }

        // Check for incoming messages
        char buffer[sizeof(Message_inter_process)];
        int rcvid = MsgReceive(Plane_channel->chid, buffer, sizeof(buffer), NULL);

        if (rcvid != -1) {
            Message* baseMsg = reinterpret_cast<Message*>(buffer);
            MessageType msgType = baseMsg->type;
            int typeValue = static_cast<int>(msgType);

            bool isValidType = (typeValue >= 0 && typeValue <= 10);
            bool isInterProcess = isValidType && (msgType != MessageType::REQUEST_POSITION);

            if (isInterProcess) {
                Message_inter_process* receivedMsg = reinterpret_cast<Message_inter_process*>(buffer);

                std::cout << "Aircraft " << id << " received inter-process message, type: "
                          << static_cast<int>(receivedMsg->type) << "\n";

                switch (receivedMsg->type) {
                    case MessageType::REQUEST_CHANGE_OF_HEADING: {
                        msg_change_heading* heading_data = reinterpret_cast<msg_change_heading*>(receivedMsg->data.data());
                        std::cout << "Aircraft " << id << " received heading change command\n";
                        std::cout << "  New velocities: VX=" << heading_data->VelocityX
                                  << " VY=" << heading_data->VelocityY
                                  << " VZ=" << heading_data->VelocityZ << "\n";
                        changeHeading(heading_data->VelocityX, heading_data->VelocityY, heading_data->VelocityZ);
                        MsgReply(rcvid, 0, NULL, 0);
                        break;
                    }

                    case MessageType::REQUEST_CHANGE_POSITION: {
                        msg_change_position* pos_data = reinterpret_cast<msg_change_position*>(receivedMsg->data.data());
                        std::cout << "Aircraft " << id << " received position change command\n";
                        std::cout << "  New position: X=" << pos_data->x
                                  << " Y=" << pos_data->y
                                  << " Z=" << pos_data->z << "\n";
                        posX = pos_data->x;
                        posY = pos_data->y;
                        posZ = pos_data->z;
                        std::cout << "Aircraft " << id << " position updated\n";
                        MsgReply(rcvid, 0, NULL, 0);
                        break;
                    }

                    case MessageType::REQUEST_CHANGE_ALTITUDE: {
                        msg_change_heading* altitude_data = reinterpret_cast<msg_change_heading*>(receivedMsg->data.data());
                        std::cout << "Aircraft " << id << " received altitude change command\n";
                        std::cout << "  New altitude: Z=" << altitude_data->altitude << "\n";
                        posZ = altitude_data->altitude;
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
                // From Radar - reply with msg_plane_info directly (no void* pointer)
                if (msgType == MessageType::REQUEST_POSITION) {
                    msg_plane_info positionData = {id, posX, posY, posZ, speedX, speedY, speedZ};
                    MsgReply(rcvid, 0, &positionData, sizeof(positionData));
                }
            }
        }

        // Wait for the next time step
        timer.waitTimer();
    }

    name_detach(Plane_channel, 0);
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
    msg.dataSize = 0;
    return msg;
}

Message Aircraft::createExitAirspaceMessage(int planeID) {
    Message msg;
    msg.type = MessageType::EXIT_AIRSPACE;
    msg.planeID = planeID;
    msg.data = NULL;
    msg.dataSize = 0;
    return msg;
}

Message Aircraft::createPositionUpdateMessage(int planeID, const msg_plane_info& info) {
    Message msg;
    msg.type = MessageType::POSITION_UPDATE;
    msg.planeID = planeID;
    msg.data = (void*)&info;
    msg.dataSize = sizeof(info);
    return msg;
}
