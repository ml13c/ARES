#pragma once

#include <cstdint>
#include <string>

namespace ares {

constexpr int kGridWidth = 20;
constexpr int kGridHeight = 10;
constexpr int kDefaultPort = 5000;

// Arbitrary SIMULATED anchor point used by the 20x10 field demonstration.
// One grid cell is one meter; positive grid y is South.
constexpr double kDemoOriginLat = 37.0;
constexpr double kDemoOriginLon = -122.0;
constexpr double kDemoMetersPerCell = 1.0;

enum class MessageType { Position, Hazard, GpsPosition, Sighting, GpsStatus,
                         FieldBoundary, PathWaypoint, NavigationStatus };
enum class NavigationState { Running, Complete, Fault };
enum class ObjectType { None, Rock, Weed, Other };

struct Message {
    MessageType message_type = MessageType::Position;
    std::uint32_t id = 0;
    int x = 0; // map grid cell only, not GPS
    int y = 0;
    ObjectType object_type = ObjectType::None;

    bool has_geo = false;
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double heading_deg = 0.0; // POS demo heading, or GPS course over ground
    bool has_heading = false; // GPS only: course may be unavailable
    double range_m = 0.0; // HAZARD only, simulated observation
    NavigationState nav_state = NavigationState::Running; // NAV only
    double relative_bearing_deg = 0.0; // HAZARD only
};

// Old format remains readable:
//   POS,id,x,y
//   HAZARD,id,ROCK|WEED|OTHER,x,y
// New extended records:
//   POS,id,x,y,latitude,longitude,heading
//   HAZARD,id,TYPE,x,y,latitude,longitude,range_m,bearing_deg
// Actual GPS telemetry (no fake grid position):
//   GPS,id,latitude,longitude[,course_over_ground_deg]
// FPGA-independent, approximate detection report (no object distance):
//   SIGHTING,id,TYPE,rover_latitude,rover_longitude
//   GPS_STATUS,NO_FIX (no fresh valid receiver fix)
// SIGHTING coordinates are the ROVER position when detected, NOT object GPS.
// Mission metadata and status (simulation only):
//   BOUNDARY,index,latitude,longitude
//   WAYPOINT,index,latitude,longitude
//   NAV,RUNNING|COMPLETE|FAULT,next_waypoint_index
// Both Pis must run the updated software for these message types.
// Each record ends with a newline; commas are not supported inside fields.
std::string encodeMessage(const Message& message);
bool decodeMessage(const std::string& line, Message& output, std::string& error);
const char* objectTypeName(ObjectType type);
const char* navigationStateName(NavigationState state);

} // namespace ares
