#ifndef RADAR_H
#define RADAR_H

#include <atomic>
#include <iostream>
#include <unordered_set>
#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include "Aircraft.h"
#include "Msg_structs.h"
#include "ATCTimer.h"

class Radar {
public:
    Radar(uint64_t& tickCounter);
    ~Radar();

    void listenAirspaceArrivalAndDeparture();
    void listenUpdatePosition();
    void writeToSharedMemory();
    void clearSharedMemory();

private:
    uint64_t& tickCounterRef;
    std::unordered_set<int> planesInAirspace;

    std::thread arrivalDepartureThread;
    std::thread positionUpdateThread;

    void addPlaneToAirspace(Message msg);
    void removePlaneFromAirspace(int id);
    void pollAirspace();
    msg_plane_info getAircraftData(int id);

    name_attach_t* radarChannel;

    std::mutex airspaceMutex;
    std::mutex bufferSwitchMutex;

    std::vector<msg_plane_info>& getActiveBuffer();
    std::vector<msg_plane_info> planesInAirspaceData[2];
    std::atomic<int> activeBufferIndex;

    ATCTimer timer;

    SharedMemory* sharedMemPtr;
    bool wasAirspaceEmpty = true;
    int shmFd = -1;
    std::atomic<bool> stopThreads;

    void shutdown();
};

#endif /* RADAR_H_ */
