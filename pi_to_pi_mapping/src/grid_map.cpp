#include "grid_map.hpp"
#include "localization.hpp"

#include <iomanip>
#include <sstream>
#include <string>
//temporary map for this test
namespace ares {

GridMap::GridMap(int width, int height) : width_(width), height_(height) {}

bool GridMap::withinBounds(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

bool GridMap::updateRover(int x, int y) {
    if (!withinBounds(x, y)) return false;
    rover_x_ = x;
    rover_y_ = y;
    has_rover_ = true;
    return true;
}

bool GridMap::addHazard(const Message& hazard) {
    if (hazard.message_type != MessageType::Hazard ||
        !withinBounds(hazard.x, hazard.y)) return false;
    hazards_[hazard.id] = hazard;
    return true;
}

bool GridMap::updateGps(const Message& position) {
    if (position.message_type != MessageType::GpsPosition ||
        !position.has_geo ||
        !validGeoPoint({position.latitude_deg, position.longitude_deg})) return false;
    live_gps_ = position;
    gps_stale_ = false;
    return true;
}

void GridMap::markGpsStale() { gps_stale_ = true; }

bool GridMap::addSighting(const Message& sighting) {
    if (sighting.message_type != MessageType::Sighting ||
        !sighting.has_geo ||
        !validGeoPoint({sighting.latitude_deg, sighting.longitude_deg})) return false;
    sightings_[sighting.id] = sighting;
    return true;
}

std::string GridMap::render() const {
    std::ostringstream out;
    out << "ARES — SIMULATED FIELD (" << width_ << " x " << height_ << ")\n";
    out << "X = rover, H = saved hazard, - = empty\n\n";

    // x increases to the right. y increases downward. Origin at top-left.
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            char marker = '-';
            for (const auto& entry : hazards_) {
                const Message& hazard = entry.second;
                if (hazard.x == x && hazard.y == y) {
                    marker = 'H';
                    break;
                }
            }
            if (has_rover_ && rover_x_ == x && rover_y_ == y) marker = 'X';
            out << marker;
        }
        out << '\n';
    }

    if (has_rover_) {
        out << "\nRover: (" << rover_x_ << ", " << rover_y_ << ")\n";
    } else {
        out << "\nRover: waiting for first position...\n";
    }
    out << "Saved hazards: " << hazards_.size() << '\n';
    for (const auto& entry : hazards_) {
        const Message& hazard = entry.second;
        out << "  #" << hazard.id << " " << objectTypeName(hazard.object_type)
            << " at grid (" << hazard.x << ", " << hazard.y << ")";
        if (hazard.has_geo) {
            out << std::fixed << std::setprecision(7)
                << " GPS (" << hazard.latitude_deg << ", "
                << hazard.longitude_deg << ")";
        }
        out << '\n';
    }
    if (live_gps_) {
        out << "\n" << (gps_stale_ ? "LAST GPS rover (STALE - DO NOT NAVIGATE): "
                                     : "LIVE GPS rover (not drawn on simulation grid): ")
            << std::fixed << std::setprecision(8)
            << live_gps_->latitude_deg << ", " << live_gps_->longitude_deg;
        if (live_gps_->has_heading)
            out << " | course over ground: " << live_gps_->heading_deg << " deg";
        else out << " | course: unavailable (stationary/unknown)";
        out << '\n';
    }
    if (gps_stale_ && !live_gps_)
        out << "\nGPS: NO VALID FIX - navigation must remain disabled\n";
    out << "Sightings (rover GPS at detection; object location UNKNOWN): "
        << sightings_.size() << '\n';
    for (const auto& entry : sightings_) {
        const auto& sighting = entry.second;
        out << "  #" << sighting.id << ' ' << objectTypeName(sighting.object_type)
            << " seen near rover GPS (" << std::fixed << std::setprecision(8)
            << sighting.latitude_deg << ", " << sighting.longitude_deg << ")\n";
    }
    return out.str();
}

} // namespace ares
