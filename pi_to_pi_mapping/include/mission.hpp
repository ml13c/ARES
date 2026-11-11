#pragma once

#include "localization.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace ares {

// Interface for a future mission planner/app: ordered latitude/longitude pairs.
// This version drives ONLY an ideal GPS simulation, never real motors.
struct Mission {
    std::vector<GeoPoint> boundary;
    std::vector<GeoPoint> waypoints;
};

bool loadMissionCsv(const std::string& filename, Mission& mission, std::string& error);
bool validateMission(const Mission& mission, double clearance_m, std::string& error);
bool insideFieldWithMargin(const Mission& mission, GeoPoint position,
                           double clearance_m);

struct SimulatedNavigation {
    GeoPoint position;
    double heading_deg = 0.0;
    std::size_t next_waypoint = 1; // first waypoint is the start
    bool complete = false;
    bool fault = false;
};

// Returns false and sets fault if moving would violate the configured geofence.
// This ideal point-mass tracker moves directly toward the next waypoint; it does
// NOT model steering, traction, camera alignment, or GNSS accuracy.
bool stepSimulatedNavigation(const Mission& mission, SimulatedNavigation& state,
                             double step_m, double clearance_m);

} // namespace ares
