#pragma once
#include <atomic>
#include <array>

// ============================================================
// Shared constants for IPC channel names and shared memory
// All channel names use the group prefix to avoid conflicts
// ============================================================
#define GROUP_PREFIX           "AU_40381796_40252823"
#define RADAR_CHANNEL_NAME     GROUP_PREFIX "_Radar"
#define COMMS_CHANNEL_NAME     GROUP_PREFIX "_Comms"
#define DISPLAY_CHANNEL_NAME   GROUP_PREFIX "_Display"
#define AIRCRAFT_CHANNEL_PREFIX GROUP_PREFIX "_"
#define SHARED_MEMORY_NAME     "/tmp/" GROUP_PREFIX "_Radar_shm"
#define PLANES_INPUT_FILE      "/tmp/40381796_40252823_planes.txt"

// ============================================================
// Message types for IPC communication
// ============================================================
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

// ============================================================
// Intra-process message (uses void* for same-process data)
// ============================================================
struct Message {
    bool isInterProcess;   // false: intra-process, true: inter-process
    MessageType type;
    int planeID;
    void* data;
    size_t dataSize;
};

// ============================================================
// Inter-process message (uses fixed buffer for cross-process data)
// ============================================================
struct Message_inter_process {
    bool isInterProcess;   // false: intra-process, true: inter-process
    MessageType type;
    int planeID;
    std::array<char, 256> data;
    size_t dataSize;
};

// ============================================================
// Aircraft position and velocity data
// ============================================================
typedef struct {
    int id;
    double positionX, positionY, positionZ;
    double velocityX, velocityY, velocityZ;
} msg_plane_info;

// ============================================================
// Command data for heading/velocity changes
// ============================================================
typedef struct {
    int id;
    double velocityX, velocityY, velocityZ;
    double altitude;
} msg_change_heading;

// ============================================================
// Command data for position changes
// ============================================================
typedef struct {
    double x, y, z;
} msg_change_position;

// ============================================================
// Shared memory layout for Radar data distribution
// ============================================================
#define SHARED_MEMORY_SIZE sizeof(SharedMemory)
#define MAX_AIRCRAFT 100

struct SharedMemory {
    msg_plane_info plane_data[MAX_AIRCRAFT];
    int count;
    std::atomic<bool> is_empty;
    uint64_t timestamp;
};
