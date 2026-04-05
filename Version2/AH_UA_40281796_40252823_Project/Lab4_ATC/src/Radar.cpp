#include "Radar.h"
#include <sys/dispatch.h>


Radar::Radar(uint64_t& tick_counter) : tick_counter_ref(tick_counter), activeBufferIndex(0), timer(1,0), stopThreads(false) {
    // Destroy any stale shared memory segment from a previous run before clearing
    shm_unlink("/tmp/AH_UA_40281796_40252823_Radar_shm");
    clearSharedMemory();
    // Start threads for listening to airspace events
    Arrival_Departure = std::thread(&Radar::ListenAirspaceArrivalAndDeparture, this);
    UpdatePosition = std::thread(&Radar::ListenUpdatePosition, this);
    Radar_channel = NULL;
}

Radar::~Radar() {
    // Join threads to ensure proper cleanup
    shutdown();
    clearSharedMemory();
}

void Radar::shutdown() {
    // Set stop flag and wait for threads to complete
    stopThreads.store(true);

    // If the channel exists, close it properly
    if (Radar_channel) {
        name_detach(Radar_channel, 0);
    }

    if (Arrival_Departure.joinable()) {
        Arrival_Departure.join();
    }
    if (UpdatePosition.joinable()) {
        UpdatePosition.join();
    }
}


// Method to get the current active buffer
std::vector<msg_plane_info>& Radar::getActiveBuffer() {
    return planesInAirspaceData[activeBufferIndex];
}

void Radar::ListenAirspaceArrivalAndDeparture() {
    Radar_channel = name_attach(NULL, "AH_UA_40281796_40252823_Radar", 0);
    if (Radar_channel == NULL) {
        std::cerr << "Failed to create channel for Radar" << std::endl;
        exit(EXIT_FAILURE);
    }
    // Simulated listening for aircraft arrivals and departures
    while (!stopThreads.load()) {
        // Replace with IPC
        Message msg;
        int rcvid = MsgReceive(Radar_channel->chid, &msg, sizeof(msg), nullptr);
        if (rcvid == -1) {
            continue;
        }

        // Reply back to the client
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

void Radar::ListenUpdatePosition() {

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
                msg_plane_info plane_info = getAircraftData(planeID);
                inactiveBuffer.emplace_back(plane_info);
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
    std::string id_str = "AH_UA_40281796_40252823_" + std::to_string(id);
    const char* ID = id_str.c_str();
    int plane_channel = name_open(ID, 0);

    if (plane_channel == -1) {
        throw std::runtime_error("Radar: Error occurred while attaching to channel");
    }

    // Prepare a request message
    Message requestMsg;
    requestMsg.type = MessageType::REQUEST_POSITION;
    requestMsg.planeID = id;
    requestMsg.data = NULL;
    requestMsg.dataSize = 0;

    // Receive directly into msg_plane_info - avoids void* cross-process pointer issue
    msg_plane_info received_info;

    if (MsgSend(plane_channel, &requestMsg, sizeof(requestMsg), &received_info, sizeof(received_info)) == -1) {
        name_close(plane_channel);
        throw std::runtime_error("Radar: Error occurred while sending request message to aircraft");
    }

    name_close(plane_channel);

    return received_info;
}

void Radar::addPlaneToAirspace(Message msg) {
    std::lock_guard<std::mutex> lock(airspaceMutex);
    int plane_data = msg.planeID;
    planesInAirspace.insert(plane_data);
    std::cout << "Plane " << msg.planeID << " added to airspace" << std::endl;
}

void Radar::removePlaneFromAirspace(int planeID) {
    std::lock_guard<std::mutex> lock(airspaceMutex);
    planesInAirspace.erase(planeID);
    std::cout << "Plane " << planeID << " removed from airspace" << std::endl;
}


void Radar::writeToSharedMemory() {
    // Open shared memory object
    shm_fd = shm_open("/tmp/AH_UA_40281796_40252823_Radar_shm", O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        std::cerr << "Failed to open shared memory" << std::endl;
        return;
    }

    // Set the size of the shared memory
    if (ftruncate(shm_fd, SHARED_MEMORY_SIZE) == -1) {
        std::cerr << "Failed to set shared memory size" << std::endl;
        close(shm_fd);
        return;
    }

    // Map the shared memory into the process address space
    SharedMemory* shared_mem = (SharedMemory*)mmap(NULL, SHARED_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_mem == MAP_FAILED) {
        std::cerr << "Failed to map shared memory" << std::endl;
        close(shm_fd);
        return;
    }

    // Lock the buffer switching mutex
    std::lock_guard<std::mutex> lock(bufferSwitchMutex);

    // Get the active buffer based on the current active index
    std::vector<msg_plane_info>& activeBuffer = getActiveBuffer();

    // Get the current timestamp
    shared_mem->timestamp = tick_counter_ref;

    // Check if activeBuffer is empty and set the flag accordingly
    if (activeBuffer.empty()) {
        std::vector<msg_plane_info>& inactiveBuffer = planesInAirspaceData[(activeBufferIndex + 1) % 2];
        if (!inactiveBuffer.empty()) {
            shared_mem->is_empty = false;
            shared_mem->count = inactiveBuffer.size();
            std::memcpy(shared_mem->plane_data, inactiveBuffer.data(), inactiveBuffer.size() * sizeof(msg_plane_info));
            inactiveBuffer.clear();
        } else {
            shared_mem->is_empty = true;
            shared_mem->count = 0;
        }
    } else {
        shared_mem->is_empty = false;
        shared_mem->count = activeBuffer.size();
        std::memcpy(shared_mem->plane_data, activeBuffer.data(), activeBuffer.size() * sizeof(msg_plane_info));
        activeBuffer.clear();
    }

    munmap(shared_mem, SHARED_MEMORY_SIZE);
    close(shm_fd);
}

void Radar::clearSharedMemory() {
    shm_fd = shm_open("/tmp/AH_UA_40281796_40252823_Radar_shm", O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        std::cerr << "Failed to open shared memory for clearing" << std::endl;
        return;
    }

    if (ftruncate(shm_fd, SHARED_MEMORY_SIZE) == -1) {
        std::cerr << "Failed to set shared memory size" << std::endl;
        close(shm_fd);
        return;
    }

    SharedMemory* sharedMemPtr = (SharedMemory*)mmap(NULL, SHARED_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (sharedMemPtr == MAP_FAILED) {
        std::cerr << "Failed to map shared memory for clearing" << std::endl;
        close(shm_fd);
        return;
    }

    std::memset(sharedMemPtr->plane_data, 0, sizeof(sharedMemPtr->plane_data));
    sharedMemPtr->count = 0;
    sharedMemPtr->is_empty = true;
    sharedMemPtr->timestamp = 0;
    sharedMemPtr->start = false;

    munmap(sharedMemPtr, SHARED_MEMORY_SIZE);
    close(shm_fd);
}
