#ifndef COMPUTER_SYSTEM_H
#define COMPUTER_SYSTEM_H

#include <iostream>
#include <thread>
#include <atomic>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <chrono>
#include <vector>

#include "Msg_structs.h"

// Collision detection distance constraints
const double CONSTRAINT_X = 3000;
const double CONSTRAINT_Y = 3000;
const double CONSTRAINT_Z = 1000;

class ComputerSystem {
public:
    ComputerSystem();
    ~ComputerSystem();

    bool startMonitoring();
    void joinThread();

private:
    void monitorAirspace();
    bool initializeSharedMemory();
    void cleanupSharedMemory();

    // Collision detection
    void checkCollision(uint64_t currentTime, std::vector<msg_plane_info> planes);
    bool checkAxes(msg_plane_info plane1, msg_plane_info plane2);

    // IPC communication
    void sendCollisionToDisplay(const Message_inter_process& msg);

    int timeConstraintCollisionFreq = 180;

    int shmFd;
    SharedMemory* sharedMem;
    std::thread monitorThread;
    std::atomic<bool> running;
};

#endif // COMPUTER_SYSTEM_H
