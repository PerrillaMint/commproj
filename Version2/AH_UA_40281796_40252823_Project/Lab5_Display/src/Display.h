#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <iostream>
#include <thread>
#include <atomic>
#include <vector>
#include <set>
#include <mutex>
#include <utility>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <unistd.h>
#include <errno.h>
#include <fstream>
#include "Msg_structs.h"

// Display channel name
#define DISPLAY_CHANNEL_NAME "40281796_40252823_Display"

// Shared memory name
#define SHARED_MEMORY_NAME "/tmp/AH_UA_40281796_40252823_Radar_shm"

// Display refresh interval in seconds (spec: every 5 seconds)
#define DISPLAY_INTERVAL_SEC 5

// History log interval in seconds (spec: every 30 seconds)
#define HISTORY_LOG_INTERVAL_SEC 30

class Display {
public:
    Display();
    ~Display();

    bool initialize();
    void run();
    void shutdown();

private:
    int shmFd;
    SharedMemory* sharedMem;
    name_attach_t* displayChannel;

    std::thread displayThread;
    std::thread collisionListenerThread;

    std::atomic<bool> running;

    std::set<int> planesInCollision;
    std::vector<std::pair<int, int>> collisionPairs;
    std::mutex collisionMutex;
    uint64_t lastCollisionTime;

    // History logging
    std::ofstream historyFile;
    uint64_t lastHistoryLogTime;

    bool initializeSharedMemory();
    bool initializeIPCChannel();

    void cleanupSharedMemory();
    void cleanupIPCChannel();

    void displayAircraft();
    void listenForCollisions();

    void printAirspaceGrid(const std::vector<msg_plane_info>& planes);

    // History file logging (spec: every 30 seconds)
    void logAirspaceHistory(const std::vector<msg_plane_info>& planes, uint64_t timestamp);
};

#endif /* DISPLAY_H_ */
