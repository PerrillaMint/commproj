#include "Display.h"
#include "ATCTimer.h"
#include <iomanip>
#include <sstream>
#include <cstring>
#include <cmath>



const double AIRSPACE_MIN_X = 0;
const double AIRSPACE_MAX_X = 100000;
const double AIRSPACE_MIN_Y = 0;
const double AIRSPACE_MAX_Y = 100000;

Display::Display() : shm_fd(-1), shared_mem(nullptr), display_channel(nullptr), running(false), lastCollisionTime(0) {}

Display::~Display() {
    shutdown();
}

bool Display::initialize() {
    if (!initializeSharedMemory()) {
        std::cerr << "Display: Failed to initialize shared memory\n";
        return false;
    }
    if (!initializeIPCChannel()) {
        std::cerr << "Display: Failed to initialize IPC channel\n";
        cleanupSharedMemory();
        return false;
    }
    running = true;
    return true;
}

bool Display::initializeSharedMemory() {
    while (true) {
        shm_fd = shm_open(SHARED_MEMORY_NAME, O_RDONLY, 0666);
        if (shm_fd == -1) {
            std::cout << "Display: Waiting for shared memory...\n";
            sleep(1);
            continue;
        }

        shared_mem = (SharedMemory*)mmap(NULL, sizeof(SharedMemory), PROT_READ, MAP_SHARED, shm_fd, 0);

        if (shared_mem == MAP_FAILED) {
            std::cerr << "Display: Failed to map shared memory\n";
            close(shm_fd);
            shm_fd = -1;
            sleep(1);
            continue;
        }

        std::cout << "Display: Shared memory initialized successfully\n";
        return true;
    }
}

bool Display::initializeIPCChannel() {
    display_channel = name_attach(NULL, DISPLAY_CHANNEL_NAME, 0);
    if (display_channel == NULL) {
        std::cerr << "Display: Failed to create channel: " << DISPLAY_CHANNEL_NAME << "\n";
        std::cerr << "  Error: " << strerror(errno) << "\n";
        return false;
    }
    std::cout << "Display: IPC channel created: " << DISPLAY_CHANNEL_NAME << "\n";
    return true;
}

void Display::cleanupSharedMemory() {
    if (shared_mem && shared_mem != MAP_FAILED) {
        munmap(shared_mem, sizeof(SharedMemory));
        shared_mem = nullptr;
    }
    if (shm_fd != -1) {
        close(shm_fd);
        shm_fd = -1;
    }
}

void Display::cleanupIPCChannel() {
    if (display_channel) {
        name_detach(display_channel, 0);
        display_channel = nullptr;
    }
}

void Display::shutdown() {
    running = false;

    if (displayThread.joinable()) {
        displayThread.join();
    }
    if (collisionListenerThread.joinable()) {
        collisionListenerThread.join();
    }

    cleanupIPCChannel();
    cleanupSharedMemory();

    std::cout << "Display: Shutdown complete\n";
}

void Display::run() {
    displayThread = std::thread(&Display::displayAircraft, this);
    collisionListenerThread = std::thread(&Display::listenForCollisions, this);

    if (displayThread.joinable()) {
        displayThread.join();
    }
    if (collisionListenerThread.joinable()) {
        collisionListenerThread.join();
    }
}

void Display::listenForCollisions() {
    std::cout << "Display: Collision listener started\n";

    while (running) {
        Message_inter_process msg;
        memset(&msg, 0, sizeof(msg));

        struct sigevent event;
        SIGEV_UNBLOCK_INIT(&event);
        uint64_t timeout = 500000000ULL;
        TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_RECEIVE, &event, &timeout, NULL);

        int rcvid = MsgReceive(display_channel->chid, &msg, sizeof(msg), NULL);

        if (rcvid == -1) continue;
        if (rcvid == 0) continue;

        int reply = 0;
        MsgReply(rcvid, 0, &reply, sizeof(reply));

        if (msg.type == MessageType::COLLISION_DETECTED) {
            size_t numPairs = msg.dataSize / sizeof(std::pair<int, int>);
            std::pair<int, int>* pairs = reinterpret_cast<std::pair<int, int>*>(msg.data.data());

            std::lock_guard<std::mutex> lock(collisionMutex);
            planesInCollision.clear();
            collisionPairs.clear();

            // Update collision time
            lastCollisionTime = shared_mem->timestamp;

            for (size_t i = 0; i < numPairs; i++) {
                planesInCollision.insert(pairs[i].first);
                planesInCollision.insert(pairs[i].second);
                collisionPairs.push_back(pairs[i]);

                //Debug Print collision warning
              // std::cout << "\n*** COLLISION WARNING: Aircraft " << pairs[i].first << " and " << pairs[i].second << " ***\n\n";
            }
        }
    }

    std::cout << "Display: Collision listener stopped\n";
}

void Display::displayAircraft() {
    ATCTimer timer(1, 0);
    std::cout << "Display: Aircraft display thread started\n";

    while (running && shared_mem->is_empty.load()) {
        std::cout << "Display: Waiting for aircraft to enter airspace...\n";
        timer.waitTimer();
    }

    while (running) {
        if (shared_mem->is_empty.load()) {

            std::cout << "\n=== AIRSPACE EMPTY - ALL AIRCRAFT HAVE DEPARTED ===\n";
            running = false;
            break;
        }

        std::vector<msg_plane_info> planes;
        int count = shared_mem->count;

        for (int i = 0; i < count && i < 100; i++) {
            planes.push_back(shared_mem->plane_data[i]);
        }


        printAirspaceGrid(planes);
        timer.waitTimer();
    }

    std::cout << "Display: Aircraft display thread stopped\n";
}




void Display::printAirspaceGrid(const std::vector<msg_plane_info>& planes) {
    std::lock_guard<std::mutex> lock(collisionMutex);

    // Clear collision warnings if no new collision for 2 seconds
    uint64_t currentTime = shared_mem->timestamp;
    if (!planesInCollision.empty() && (currentTime - lastCollisionTime) > 2) {
        planesInCollision.clear();
        collisionPairs.clear();
    }


    // Print aircraft details
    std::cout << " Aircraft Details:\n";
    std::cout << "-------------------------------------------------------------------------\n";

    for (const auto& plane : planes) {
        bool inCollision = planesInCollision.find(plane.id) != planesInCollision.end();

        // Find collision partner(s)
        std::string collisionInfo = "";
        if (inCollision) {
            for (const auto& pair : collisionPairs) {
                if (pair.first == plane.id) {
                    collisionInfo = " COLLISION WITH PLANE " + std::to_string(pair.second);
                } else if (pair.second == plane.id) {
                    collisionInfo = " COLLISION WITH PLANE " + std::to_string(pair.first);
                }
            }
        }

        std::cout << "  ID:" << std::setw(2) << plane.id
                  << " Pos(" << std::setw(6) << (int)plane.PositionX << ","
                  << std::setw(6) << (int)plane.PositionY << ","
                  << std::setw(6) << (int)plane.PositionZ << ")"
                  << " Vel(" << std::setw(4) << (int)plane.VelocityX << ","
                  << std::setw(4) << (int)plane.VelocityY << ","
                  << std::setw(4) << (int)plane.VelocityZ << ")";

        if (inCollision) {
            std::cout << collisionInfo;
        }
        std::cout << "\n";
    }

    std::cout << "\n";
}
