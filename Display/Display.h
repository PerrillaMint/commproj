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

    bool initializeSharedMemory();
    bool initializeIPCChannel();

    void cleanupSharedMemory();
    void cleanupIPCChannel();

    void displayAircraft();
    void listenForCollisions();

    void printAirspaceGrid(const std::vector<msg_plane_info>& planes);
};

#endif /* DISPLAY_H_ */
