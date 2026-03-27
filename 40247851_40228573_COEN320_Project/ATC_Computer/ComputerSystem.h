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

const double CONSTRAINT_X = 3000;
const double CONSTRAINT_Y = 3000;
const double CONSTRAINT_Z = 1000;

#include "Msg_structs.h"

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

    //Collsion detection
    void checkCollision(uint64_t currentTime, std::vector<msg_plane_info> planes);
    bool checkAxes(msg_plane_info plane1, msg_plane_info plane2);
    bool sameSpeed(double peed1, double speed2);

    //Handle messages from operator
    void processMessage();
    void sendMessagesToComms(const Message& msg);
    void handleTimeConstraintChange(const Message& msg);
    void sendCollisionToDisplay(const Message_inter_process& msg);

    int timeConstraintCollisionFreq = 180;



    int shm_fd;
    SharedMemory* shared_mem;
    std::thread monitorThread;
    std::thread monitorOperatorInput;
    std::atomic<bool> running;

    bool listen = true;
};

#endif // COMPUTER_SYSTEM_H
