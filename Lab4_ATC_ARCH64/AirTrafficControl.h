#ifndef AIRTRAFFICCONTROL_H
#define AIRTRAFFICCONTROL_H

#include "Aircraft.h"
#include <vector>
#include <thread>
#include <string>

struct PlaneData {
    int arrivalTime;
    int id;
    int posX, posY, posZ;
    int speedX, speedY, speedZ;
};

class AirTrafficControl {
public:
    AirTrafficControl();
    ~AirTrafficControl();

    void readPlanesFromFile(const std::string& fileName);
    void startPlanes();
    bool areAllPlanesFinished() const;

private:
    std::vector<Aircraft*> planes;
    std::vector<PlaneData> planeData;
    bool allPlanesFinished = false;
};

#endif // AIRTRAFFICCONTROL_H
