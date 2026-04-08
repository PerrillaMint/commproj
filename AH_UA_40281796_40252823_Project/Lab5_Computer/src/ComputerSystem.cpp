#include "ComputerSystem.h"
#include "ATCTimer.h"
#include <ctime>
#include <iomanip>
#include <cmath>
#include <sys/dispatch.h>
#include "Msg_structs.h"
#include <cstring>

#define display_channel_name "40281796_40252823_Display"

// Initialize the static atomic time constraint (default 180 seconds = 3 minutes)
std::atomic<int> ComputerSystem::timeConstraintCollisionFreq(180);

ComputerSystem::ComputerSystem() : shm_fd(-1), shared_mem(nullptr), running(false) {}

ComputerSystem::~ComputerSystem() {
    joinThread();
    cleanupSharedMemory();
}

bool ComputerSystem::initializeSharedMemory() {
    while (true) {
        shm_fd = shm_open("/tmp/AH_UA_40281796_40252823_Radar_shm", O_RDONLY, 0666);

        if (shm_fd == -1) {
            std::cerr << "Failed to open shared memory, retrying..." << std::endl;
            sleep(1);
            continue;
        }

        shared_mem = (SharedMemory*)mmap(NULL, sizeof(SharedMemory), PROT_READ, MAP_SHARED, shm_fd, 0);

        if (shared_mem == MAP_FAILED) {
            std::cerr << "Failed to map shared memory, retrying..." << std::endl;
            close(shm_fd);
            shm_fd = -1;
            sleep(1);
            continue;
        }

        return true;
    }
}

void ComputerSystem::cleanupSharedMemory() {
    if (shared_mem && shared_mem != MAP_FAILED) {
        munmap(shared_mem, sizeof(SharedMemory));
    }
    if (shm_fd != -1) {
        close(shm_fd);
    }
}

bool ComputerSystem::startMonitoring() {
    if (initializeSharedMemory()) {
        running = true;
        std::cout << "Starting monitoring thread." << std::endl;
        monitorThread = std::thread(&ComputerSystem::monitorAirspace, this);
        return true;
    } else {
        std::cerr << "Failed to initialize shared memory. Monitoring not started.\n";
        return false;
    }
}

void ComputerSystem::joinThread() {
    if (monitorThread.joinable()) {
        monitorThread.join();
    }
}

void ComputerSystem::monitorAirspace() {
    ATCTimer timer(1, 0);
    std::vector<msg_plane_info> plane_data_vector;
    uint64_t timestamp;

    // Wait until planes are present - plain bool read, no .load()
    while (shared_mem->is_empty) {
        std::cout << "Waiting for planes in airspace...\n";
        timer.waitTimer();
    }

    while (running) {
        // Plain bool read, no .load()
        if (shared_mem->is_empty) {
            std::cout << "No planes in airspace. Stopping monitoring.\n";
            running = false;
            break;
        } else {
            plane_data_vector.clear();
            timestamp = shared_mem->timestamp;

            for (int i = 0; i < shared_mem->count; ++i) {
                const msg_plane_info& plane = shared_mem->plane_data[i];
                plane_data_vector.push_back(plane);
            }
        }

        if (plane_data_vector.size() > 1)
            checkCollision(timestamp, plane_data_vector);
        else
            //std::cout << "No collision possible with single plane\n";

        timer.waitTimer();
    }

    std::cout << "Exiting monitoring loop." << std::endl;
}

void ComputerSystem::checkCollision(uint64_t currentTime, std::vector<msg_plane_info> planes) {
    std::vector<CollisionPair> collisionPairs;

    for (size_t i = 0; i < planes.size(); i++) {
        for (size_t j = i + 1; j < planes.size(); j++) {
            int result = checkAxes(planes[i], planes[j]);
            if (result == 0) {
                collisionPairs.push_back({planes[i].id, planes[j].id, CollisionSeverity::COLLISION});
            } else if (result == 1) {
                collisionPairs.push_back({planes[i].id, planes[j].id, CollisionSeverity::WARNING});
            }
        }
    }

    if (!collisionPairs.empty()) {
        Message_inter_process msg_to_send;

        size_t numPairs = collisionPairs.size();
        size_t dataSize = numPairs * sizeof(CollisionPair);

        msg_to_send.header = true;
        msg_to_send.planeID = -1;
        msg_to_send.type = MessageType::COLLISION_DETECTED;
        msg_to_send.dataSize = dataSize;
        std::memcpy(msg_to_send.data.data(), collisionPairs.data(), dataSize);

        sendCollisionToDisplay(msg_to_send);
    }
}

int ComputerSystem::checkAxes(msg_plane_info plane1, msg_plane_info plane2) {
    // Check current separation
    double deltaX = std::abs(plane1.PositionX - plane2.PositionX);
    double deltaY = std::abs(plane1.PositionY - plane2.PositionY);
    double deltaZ = std::abs(plane1.PositionZ - plane2.PositionZ);

    if (deltaX < CONSTRAINT_X && deltaY < CONSTRAINT_Y && deltaZ < CONSTRAINT_Z) {
        return 0; // Current collision
    }

    // Predict future positions based on time constraint
    int tcf = timeConstraintCollisionFreq.load();
    double futureX1 = plane1.PositionX + plane1.VelocityX * tcf;
    double futureY1 = plane1.PositionY + plane1.VelocityY * tcf;
    double futureZ1 = plane1.PositionZ + plane1.VelocityZ * tcf;

    double futureX2 = plane2.PositionX + plane2.VelocityX * tcf;
    double futureY2 = plane2.PositionY + plane2.VelocityY * tcf;
    double futureZ2 = plane2.PositionZ + plane2.VelocityZ * tcf;

    double futureDeltaX = std::abs(futureX1 - futureX2);
    double futureDeltaY = std::abs(futureY1 - futureY2);
    double futureDeltaZ = std::abs(futureZ1 - futureZ2);

    if (futureDeltaX < CONSTRAINT_X && futureDeltaY < CONSTRAINT_Y && futureDeltaZ < CONSTRAINT_Z) {
        return 1; // Predicted collision (warning)
    }

    return -1; // No collision
}

void ComputerSystem::sendCollisionToDisplay(const Message_inter_process& msg) {
    int display_channel = name_open(display_channel_name, 0);
    if (display_channel == -1) {
        throw std::runtime_error("Computer system: Error occurred while attaching to display");
    }
    int reply;
    int status = MsgSend(display_channel, &msg, sizeof(msg), &reply, sizeof(reply));
    if (status == -1) {
        perror("Computer system: Error occurred while sending message to display channel");
    }
    name_close(display_channel);
}
