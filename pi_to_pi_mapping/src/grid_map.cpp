#include "grid_map.hpp"

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
            << " at (" << hazard.x << ", " << hazard.y << ")\n";
    }
    return out.str();
}

} // namespace ares
