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
#include "Msg_structs.h"

// Display channel name
#define DISPLAY_CHANNEL_NAME "40247851_40228573_Display"

// Shared memory name
#define SHARED_MEMORY_NAME "/tmp/AH_40247851_40228573_Radar_shm"

class Display {
public:
    Display();
    ~Display();


    bool initialize();
    void run();
    void shutdown();

private:

    int shm_fd;
    SharedMemory* shared_mem;

    name_attach_t* display_channel;


    std::thread displayThread;
    std::thread collisionListenerThread;

    std::atomic<bool> running;

    std::set<int> planesInCollision;
    std::vector<std::pair<int, int>> collisionPairs;
    std::mutex collisionMutex;
    uint64_t lastCollisionTime;


    bool initializeSharedMemory();
    bool initializeIPCChannel();


    void cleanupSharedMemory();
    void cleanupIPCChannel();


    void displayAircraft();
    void listenForCollisions();


    void printAirspaceGrid(const std::vector<msg_plane_info>& planes);
    void clearScreen();


};

#endif /* DISPLAY_H_ */
