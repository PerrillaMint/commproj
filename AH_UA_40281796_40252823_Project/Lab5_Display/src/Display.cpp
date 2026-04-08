#include "Display.h"
#include "ATCTimer.h"
#include <iomanip>
#include <sstream>
#include <cstring>
#include <cmath>
#include <ctime>

Display::Display() : shmFd(-1), sharedMem(nullptr), displayChannel(nullptr), running(false), lastCollisionTime(0), lastHistoryLogTime(0) {}

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

    // Open history log file (spec: store airspace every 30 seconds)
    historyFile.open("airspace_history.log", std::ios::out | std::ios::app);
    if (!historyFile.is_open()) {
        std::cerr << "Display: Warning - Could not open history log file\n";
    }

    running = true;
    return true;
}

bool Display::initializeSharedMemory() {
    bool initialized = false;
    while (!initialized) {
        shmFd = shm_open(SHARED_MEMORY_NAME, O_RDONLY, 0666);
        if (shmFd == -1) {
            std::cout << "Display: Waiting for shared memory...\n";
            sleep(1);
            continue;
        }

        size_t shm_size = sizeof(SharedMemory);
        sharedMem = (SharedMemory*)mmap(NULL, shm_size, PROT_READ, MAP_SHARED, shmFd, 0);
        if (sharedMem == MAP_FAILED) {
            std::cerr << "Display: Failed to map shared memory\n";
            close(shmFd);
            shmFd = -1;
            sleep(1);
            continue;
        }

        initialized = true;
    }

    std::cout << "Display: Shared memory initialized successfully\n";
    return true;
}

bool Display::initializeIPCChannel() {
    displayChannel = name_attach(NULL, DISPLAY_CHANNEL_NAME, 0);
    if (displayChannel == NULL) {
        std::cerr << "Display: Failed to create channel: " << DISPLAY_CHANNEL_NAME << "\n";
        std::cerr << "  Error: " << strerror(errno) << "\n";
        return false;
    }
    std::cout << "Display: IPC channel created: " << DISPLAY_CHANNEL_NAME << "\n";
    return true;
}

void Display::cleanupSharedMemory() {
    if (sharedMem && sharedMem != MAP_FAILED) {
        munmap(sharedMem, sizeof(SharedMemory));
        sharedMem = nullptr;
    }
    if (shmFd != -1) {
        close(shmFd);
        shmFd = -1;
    }
}

void Display::cleanupIPCChannel() {
    if (displayChannel) {
        name_detach(displayChannel, 0);
        displayChannel = nullptr;
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

    if (historyFile.is_open()) {
        historyFile.close();
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

        int rcvid = MsgReceive(displayChannel->chid, &msg, sizeof(msg), NULL);
        if (rcvid <= 0) continue;

        int reply = 0;
        MsgReply(rcvid, 0, &reply, sizeof(reply));

        if (msg.type != MessageType::COLLISION_DETECTED) continue;

        size_t numPairs = msg.dataSize / sizeof(CollisionPair);
        CollisionPair* pairs = reinterpret_cast<CollisionPair*>(msg.data.data());

        std::lock_guard<std::mutex> lock(collisionMutex);
        planesInCollision.clear();
        collisionPairs.clear();
        lastCollisionTime = sharedMem->timestamp;

        for (size_t i = 0; i < numPairs; i++) {
            planesInCollision.insert(pairs[i].plane1);
            planesInCollision.insert(pairs[i].plane2);
            collisionPairs.push_back(pairs[i]);
        }
    }

    std::cout << "Display: Collision listener stopped\n";
}

void Display::displayAircraft() {
    ATCTimer timer(DISPLAY_INTERVAL_SEC, 0);
    std::cout << "Display: Aircraft display thread started\n";

    while (running && sharedMem->is_empty) {
        std::cout << "Display: Waiting for aircraft to enter airspace...\n";
        timer.waitTimer();
    }

    while (running) {
        if (sharedMem->is_empty) {
            std::cout << "AIRSPACE EMPTY, ALL AIRCRAFT HAVE DEPARTED\n";
            running = false;
            break;
        }

        uint64_t timestamp = sharedMem->timestamp;
        int count = sharedMem->count;

        std::vector<msg_plane_info> planes;
        planes.reserve(count);
        for (int i = 0; i < count && i < 100; i++) {
            planes.push_back(sharedMem->plane_data[i]);
        }

        printAirspaceGrid(planes);

        if (timestamp - lastHistoryLogTime >= HISTORY_LOG_INTERVAL_SEC) {
            logAirspaceHistory(planes, timestamp);
            lastHistoryLogTime = timestamp;
        }

        timer.waitTimer();
    }

    std::cout << "Display: Aircraft display thread stopped\n";
}

void Display::printAirspaceGrid(const std::vector<msg_plane_info>& planes) {
    std::lock_guard<std::mutex> lock(collisionMutex);

    uint64_t currentTime = sharedMem->timestamp;
    if (!planesInCollision.empty() && (currentTime - lastCollisionTime) > 2) {
        planesInCollision.clear();
        collisionPairs.clear();
    }

    auto getCollisionInfo = [&](int id) -> std::string {
        for (const auto& pair : collisionPairs) {
            int other = -1;
            if (pair.plane1 == id) other = pair.plane2;
            else if (pair.plane2 == id) other = pair.plane1;

            if (other != -1) {
                if (pair.severity == CollisionSeverity::COLLISION)
                    return " COLLISION WITH PLANE " + std::to_string(other);
                else
                    return " WARNING: PREDICTED COLLISION WITH PLANE " + std::to_string(other);
            }
        }
        return "";
    };

    std::cout << " Aircraft Details:\n";
    std::cout << "****************************************************************\n";

    for (const auto& plane : planes) {
        bool inCollision = planesInCollision.find(plane.id) != planesInCollision.end();

        std::cout << "  ID:" << std::setw(2) << plane.id
                  << " Pos(" << std::setw(6) << (int)plane.PositionX << ","
                  << std::setw(6) << (int)plane.PositionY << ","
                  << std::setw(6) << (int)plane.PositionZ << ")"
                  << " Vel(" << std::setw(4) << (int)plane.VelocityX << ","
                  << std::setw(4) << (int)plane.VelocityY << ","
                  << std::setw(4) << (int)plane.VelocityZ << ")";

        if (inCollision)
            std::cout << getCollisionInfo(plane.id);

        std::cout << "\n";
    }

    std::cout << "\n";
}

void Display::logAirspaceHistory(const std::vector<msg_plane_info>& planes, uint64_t timestamp) {
    if (!historyFile.is_open()) return;

    auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char timeBuf[64];
    std::strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%d %H:%M:%S", std::localtime(&now));

    historyFile << "=== Airspace Snapshot | Wall-clock: " << timeBuf
                << " | Sim-time: " << timestamp << "s"
                << " | Aircraft count: " << planes.size() << " ===\n";

    for (const auto& plane : planes) {
        historyFile << "  ID=" << plane.id
                    << " X=" << (int)plane.PositionX
                    << " Y=" << (int)plane.PositionY
                    << " Z=" << (int)plane.PositionZ
                    << " Vx=" << (int)plane.VelocityX
                    << " Vy=" << (int)plane.VelocityY
                    << " Vz=" << (int)plane.VelocityZ
                    << "\n";
    }

    {
        std::lock_guard<std::mutex> lock(collisionMutex);
        if (!collisionPairs.empty()) {
            historyFile << "  COLLISIONS: ";
            for (const auto& pair : collisionPairs) {
                std::string label = (pair.severity == CollisionSeverity::COLLISION)
                    ? "COLLISION" : "WARNING";
                historyFile << label << "(" << pair.plane1 << "," << pair.plane2 << ") ";
            }
            historyFile << "\n";
        }
    }

    historyFile << "\n";
    historyFile.flush();
}
