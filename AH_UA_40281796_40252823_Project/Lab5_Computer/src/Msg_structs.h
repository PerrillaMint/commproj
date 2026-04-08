#pragma once
#include <array>

enum class MessageType {
    ENTER_AIRSPACE,
    EXIT_AIRSPACE,
    POSITION_UPDATE,
    REQUEST_POSITION,
    REQUEST_CHANGE_OF_HEADING,
    REQUEST_CHANGE_POSITION,
    REQUEST_CHANGE_ALTITUDE,
    REQUEST_AUGMENTED_INFO,
    CHANGE_TIME_CONSTRAINT_COLLISIONS,
    EXIT,
    COLLISION_DETECTED
};

struct Message {
    bool header; //intra process; 1: interprocess
    MessageType type;
    int planeID;
    void* data;  // Pointer to the message data
    size_t dataSize;  // Size of the serialized data
};

typedef struct {
    int id;
    double PositionX, PositionY, PositionZ, VelocityX, VelocityY, VelocityZ;
} msg_plane_info;

typedef struct {
    int ID;
    double VelocityX, VelocityY, VelocityZ;
    double altitude;
} msg_change_heading;

typedef struct {
    double x, y, z;
} msg_change_position;

// Shared memory structure
struct SharedMemory {
    msg_plane_info plane_data[100];
    int count;              // Number of planes in the buffer
    volatile bool is_empty; // Plain volatile bool - safe for cross-process shared memory
    bool start;
    uint64_t timestamp;     // Timestamp of the last write
};

struct Message_inter_process {
    bool header; //intra process; 1: interprocess
    MessageType type;
    int planeID;
    std::array<char, 256> data;  // Message data buffer
    size_t dataSize;             // Size of the serialized data
};

// Collision severity levels
enum class CollisionSeverity {
    COLLISION,  // Currently colliding (within constraint thresholds)
    WARNING     // Predicted to collide within time constraint
};

// Collision pair with severity info for Display communication
struct CollisionPair {
    int plane1;
    int plane2;
    CollisionSeverity severity;
};
