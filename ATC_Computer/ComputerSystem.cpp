#include "ComputerSystem.h"
#include "ATCTimer.h"
#include <ctime>
#include <iomanip>
#include <cmath>
#include <sys/dispatch.h>
#include <cstring>
#include <sched.h>

ComputerSystem::ComputerSystem() : shmFd(-1), sharedMem(nullptr), running(false) {}

ComputerSystem::~ComputerSystem() {
    joinThread();
    cleanupSharedMemory();
}

bool ComputerSystem::initializeSharedMemory() {
    while (true) {
        shmFd = shm_open(SHARED_MEMORY_NAME, O_RDONLY, 0666);

        if (shmFd == -1) {
            std::cerr << "Failed to open shared memory, retrying..." << std::endl;
            sleep(1);
            continue;
        }

        sharedMem = (SharedMemory*)mmap(NULL, sizeof(SharedMemory), PROT_READ, MAP_SHARED, shmFd, 0);

        if (sharedMem == MAP_FAILED) {
            std::cerr << "Failed to map shared memory, retrying..." << std::endl;
            close(shmFd);
            shmFd = -1;
            sleep(1);
            continue;
        }

        return true;
    }
}

void ComputerSystem::cleanupSharedMemory() {
    if (sharedMem && sharedMem != MAP_FAILED) {
        munmap(sharedMem, sizeof(SharedMemory));
    }
    if (shmFd != -1) {
        close(shmFd);
    }
}

bool ComputerSystem::startMonitoring() {
    if (initializeSharedMemory()) {
        running = true;
        monitorThread = std::thread(&ComputerSystem::monitorAirspace, this);

        // Set collision detection thread to highest priority (safety-critical)
        struct sched_param param;
        param.sched_priority = 20;
        pthread_setschedparam(monitorThread.native_handle(), SCHED_FIFO, &param);

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
    std::vector<msg_plane_info> planeDataVector;
    uint64_t timestamp;

    // Wait for aircraft to appear
    while (sharedMem->is_empty.load()) {
        std::cout << "Waiting for planes in airspace...\n";
        timer.waitTimer();
    }

    while (running) {
        if (sharedMem->is_empty.load()) {
            std::cout << "No planes in airspace. Stopping monitoring.\n";
            running = false;
            break;
        } else {
            planeDataVector.clear();
            timestamp = sharedMem->timestamp;

            for (int i = 0; i < sharedMem->count; ++i) {
                planeDataVector.push_back(sharedMem->plane_data[i]);
            }
        }

        if (planeDataVector.size() > 1)
            checkCollision(timestamp, planeDataVector);

        timer.waitTimer();
    }

    std::cout << "Exiting monitoring loop." << std::endl;
}

void ComputerSystem::checkCollision(uint64_t currentTime, std::vector<msg_plane_info> planes) {
    std::vector<std::pair<int, int>> collisionPairs;

    for (size_t i = 0; i < planes.size(); i++) {
        for (size_t j = i + 1; j < planes.size(); j++) {
            if (checkAxes(planes[i], planes[j])) {
                collisionPairs.emplace_back(planes[i].id, planes[j].id);
            }
        }
    }

    if (!collisionPairs.empty()) {
        Message_inter_process msg;

        size_t numPairs = collisionPairs.size();
        size_t dataSize = numPairs * sizeof(std::pair<int, int>);

        msg.isInterProcess = true;
        msg.planeID = -1;
        msg.type = MessageType::COLLISION_DETECTED;
        msg.dataSize = dataSize;
        std::memcpy(msg.data.data(), collisionPairs.data(), dataSize);

        sendCollisionToDisplay(msg);
    }
}

bool ComputerSystem::checkAxes(msg_plane_info plane1, msg_plane_info plane2) {
    // Check current positions
    double deltaX = std::abs(plane1.positionX - plane2.positionX);
    double deltaY = std::abs(plane1.positionY - plane2.positionY);
    double deltaZ = std::abs(plane1.positionZ - plane2.positionZ);

    if (deltaX < CONSTRAINT_X && deltaY < CONSTRAINT_Y && deltaZ < CONSTRAINT_Z) {
        return true;
    }

    // Predict future positions
    double futureX1 = plane1.positionX + plane1.velocityX * timeConstraintCollisionFreq;
    double futureY1 = plane1.positionY + plane1.velocityY * timeConstraintCollisionFreq;
    double futureZ1 = plane1.positionZ + plane1.velocityZ * timeConstraintCollisionFreq;

    double futureX2 = plane2.positionX + plane2.velocityX * timeConstraintCollisionFreq;
    double futureY2 = plane2.positionY + plane2.velocityY * timeConstraintCollisionFreq;
    double futureZ2 = plane2.positionZ + plane2.velocityZ * timeConstraintCollisionFreq;

    double futureDeltaX = std::abs(futureX1 - futureX2);
    double futureDeltaY = std::abs(futureY1 - futureY2);
    double futureDeltaZ = std::abs(futureZ1 - futureZ2);

    if (futureDeltaX < CONSTRAINT_X && futureDeltaY < CONSTRAINT_Y && futureDeltaZ < CONSTRAINT_Z) {
        return true;
    }

    return false;
}

void ComputerSystem::sendCollisionToDisplay(const Message_inter_process& msg) {
    int displayConnectionId = name_open(DISPLAY_CHANNEL_NAME, 0);
    if (displayConnectionId == -1) {
        throw std::runtime_error("Computer system: Error occurred while attaching to display");
    }

    int reply;
    int status = MsgSend(displayConnectionId, &msg, sizeof(msg), &reply, sizeof(reply));
    if (status == -1) {
        perror("Computer system: Error occurred while sending message to display channel");
    }

    name_close(displayConnectionId);
}
