#include "Radar.h"
#include <sys/dispatch.h>
#include <sched.h>

Radar::Radar(uint64_t& tickCounter)
    : tickCounterRef(tickCounter), activeBufferIndex(0), timer(1, 0), stopThreads(false) {
    clearSharedMemory();
    radarChannel = NULL;

    // Start Radar threads with higher priority than aircraft (safety-critical)
    arrivalDepartureThread = std::thread(&Radar::listenAirspaceArrivalAndDeparture, this);
    positionUpdateThread = std::thread(&Radar::listenUpdatePosition, this);

    // Set Radar threads to higher priority (15) than aircraft threads (10)
    struct sched_param param;
    param.sched_priority = 15;
    pthread_setschedparam(arrivalDepartureThread.native_handle(), SCHED_FIFO, &param);
    pthread_setschedparam(positionUpdateThread.native_handle(), SCHED_FIFO, &param);
}

Radar::~Radar() {
    shutdown();
    clearSharedMemory();
}

void Radar::shutdown() {
    stopThreads.store(true);

    if (radarChannel) {
        name_detach(radarChannel, 0);
    }

    if (arrivalDepartureThread.joinable()) {
        arrivalDepartureThread.join();
    }
    if (positionUpdateThread.joinable()) {
        positionUpdateThread.join();
    }
}

std::vector<msg_plane_info>& Radar::getActiveBuffer() {
    return planesInAirspaceData[activeBufferIndex];
}

void Radar::listenAirspaceArrivalAndDeparture() {
    radarChannel = name_attach(NULL, RADAR_CHANNEL_NAME, 0);
    if (radarChannel == NULL) {
        std::cerr << "Failed to create channel for Radar" << std::endl;
        exit(EXIT_FAILURE);
    }

    while (!stopThreads.load()) {
        Message msg;
        int rcvid = MsgReceive(radarChannel->chid, &msg, sizeof(msg), nullptr);
        if (rcvid == -1) {
            continue;
        }

        int msg_ret = msg.planeID;
        MsgReply(rcvid, 0, &msg_ret, sizeof(msg_ret));

        switch (msg.type) {
        case MessageType::ENTER_AIRSPACE:
            addPlaneToAirspace(msg);
            break;
        case MessageType::EXIT_AIRSPACE:
            removePlaneFromAirspace(msg.planeID);
            break;
        default:
            break;
        }
    }
}

void Radar::listenUpdatePosition() {
    while (!stopThreads.load()) {
        timer.waitTimer();

        if (!planesInAirspace.empty()) {
            pollAirspace();
            writeToSharedMemory();
            wasAirspaceEmpty = false;
        } else if (!wasAirspaceEmpty) {
            writeToSharedMemory();
            wasAirspaceEmpty = true;
        }
    }
}

void Radar::pollAirspace() {
    airspaceMutex.lock();
    std::unordered_set<int> planesToPoll = planesInAirspace;
    airspaceMutex.unlock();

    int inactiveBufferIndex = (activeBufferIndex + 1) % 2;
    std::vector<msg_plane_info>& inactiveBuffer = planesInAirspaceData[inactiveBufferIndex];
    inactiveBuffer.clear();

    for (int planeID : planesToPoll) {
        airspaceMutex.lock();
        bool isPlaneInAirspace = planesInAirspace.find(planeID) != planesInAirspace.end();
        airspaceMutex.unlock();

        if (isPlaneInAirspace) {
            try {
                msg_plane_info planeInfo = getAircraftData(planeID);
                inactiveBuffer.emplace_back(planeInfo);
            } catch (const std::exception& e) {
                continue;
            }
        }

        {
            std::lock_guard<std::mutex> lock(bufferSwitchMutex);
            activeBufferIndex = inactiveBufferIndex;
        }
    }
}

msg_plane_info Radar::getAircraftData(int id) {
    std::string channelName = std::string(AIRCRAFT_CHANNEL_PREFIX) + std::to_string(id);
    int planeConnectionId = name_open(channelName.c_str(), 0);

    if (planeConnectionId == -1) {
        throw std::runtime_error("Radar: Error occurred while attaching to channel");
    }

    Message requestMsg;
    requestMsg.type = MessageType::REQUEST_POSITION;
    requestMsg.planeID = id;
    requestMsg.data = NULL;

    Message receiveMessage;

    if (MsgSend(planeConnectionId, &requestMsg, sizeof(requestMsg), &receiveMessage, sizeof(receiveMessage)) == -1) {
        name_close(planeConnectionId);
        throw std::runtime_error("Radar: Error occurred while sending request message to aircraft");
    }

    msg_plane_info receivedInfo = *static_cast<msg_plane_info*>(receiveMessage.data);

    name_close(planeConnectionId);

    return receivedInfo;
}

void Radar::addPlaneToAirspace(Message msg) {
    std::lock_guard<std::mutex> lock(airspaceMutex);
    planesInAirspace.insert(msg.planeID);
    std::cout << "Plane " << msg.planeID << " added to airspace" << std::endl;
}

void Radar::removePlaneFromAirspace(int planeID) {
    std::lock_guard<std::mutex> lock(airspaceMutex);
    planesInAirspace.erase(planeID);
    std::cout << "Plane " << planeID << " removed from airspace" << std::endl;
}

void Radar::writeToSharedMemory() {
    shmFd = shm_open(SHARED_MEMORY_NAME, O_CREAT | O_RDWR, 0666);
    if (shmFd == -1) {
        std::cerr << "Failed to open shared memory" << std::endl;
        return;
    }

    if (ftruncate(shmFd, SHARED_MEMORY_SIZE) == -1) {
        std::cerr << "Failed to set shared memory size" << std::endl;
        close(shmFd);
        return;
    }

    SharedMemory* shared_mem = (SharedMemory*)mmap(NULL, SHARED_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shmFd, 0);
    if (shared_mem == MAP_FAILED) {
        std::cerr << "Failed to map shared memory" << std::endl;
        close(shmFd);
        return;
    }

    std::lock_guard<std::mutex> lock(bufferSwitchMutex);

    std::vector<msg_plane_info>& activeBuffer = getActiveBuffer();

    shared_mem->timestamp = tickCounterRef;

    if (activeBuffer.empty()) {
        std::vector<msg_plane_info>& inactiveBuffer = planesInAirspaceData[(activeBufferIndex + 1) % 2];
        if (!inactiveBuffer.empty()) {
            shared_mem->is_empty.store(false);
            shared_mem->count = inactiveBuffer.size();
            std::memcpy(shared_mem->plane_data, inactiveBuffer.data(), inactiveBuffer.size() * sizeof(msg_plane_info));
            inactiveBuffer.clear();
        } else {
            shared_mem->is_empty.store(true);
            shared_mem->count = 0;
        }
    } else {
        shared_mem->is_empty.store(false);
        shared_mem->count = activeBuffer.size();
        std::memcpy(shared_mem->plane_data, activeBuffer.data(), activeBuffer.size() * sizeof(msg_plane_info));
        activeBuffer.clear();
    }

    munmap(shared_mem, SHARED_MEMORY_SIZE);
    close(shmFd);
}

void Radar::clearSharedMemory() {
    shmFd = shm_open(SHARED_MEMORY_NAME, O_RDWR, 0666);
    if (shmFd == -1) {
        std::cerr << "Failed to open shared memory for clearing" << std::endl;
        return;
    }

    if (ftruncate(shmFd, SHARED_MEMORY_SIZE) == -1) {
        std::cerr << "Failed to set shared memory size" << std::endl;
        close(shmFd);
        return;
    }

    SharedMemory* shmPtr = (SharedMemory*)mmap(NULL, SHARED_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shmFd, 0);
    if (shmPtr == MAP_FAILED) {
        std::cerr << "Failed to map shared memory for clearing" << std::endl;
        close(shmFd);
        return;
    }

    std::memset(shmPtr->plane_data, 0, sizeof(shmPtr->plane_data));
    shmPtr->count = 0;
    shmPtr->is_empty.store(true);
    shmPtr->timestamp = 0;

    munmap(shmPtr, SHARED_MEMORY_SIZE);
    close(shmFd);
}
