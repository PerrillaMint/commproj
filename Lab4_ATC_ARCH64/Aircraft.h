#ifndef AIRCRAFT_H_
#define AIRCRAFT_H_

#include <iostream>
#include <sys/dispatch.h>
#include <thread>
#include "Msg_structs.h"

typedef struct {
    int lower_x_boundary;
    int upper_x_boundary;
    int lower_y_boundary;
    int upper_y_boundary;
    int lower_z_boundary;
    int upper_z_boundary;
} airspace_struct;

class Aircraft {
public:
    Aircraft(int id, double x, double y, double z, double sx, double sy, double sz, int arrivalTime);
    ~Aircraft();

    void printInitialAircraftData() const;
    void printAircraft() const;
    int updatePosition();
    int getArrivalTime();
    int getID();
    void changeHeading(double vx, double vy, double vz);

    pthread_t threadId;

private:
    int id;
    double posX, posY, posZ;
    double speedX, speedY, speedZ;
    int arrivalTime;
    int messageId;
    bool inAirspace;
    int radarConnectionId;
    airspace_struct airspace;

    Message createEnterAirspaceMessage(int planeID);
    Message createExitAirspaceMessage(int planeID);
    Message createPositionUpdateMessage(int planeID, const msg_plane_info& info);
};

#endif /* AIRCRAFT_H_ */
